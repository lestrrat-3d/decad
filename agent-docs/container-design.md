# Container Design

A `Container` is a named group inside one `Document`. It holds sketches, planes and bodies and
nests inside other containers. It is bookkeeping only: it has no transform, it instances nothing,
and it changes no geometry. `agent-docs/api-design.md` §13 keeps assemblies (`Component` and
`Occurrence` instancing and the graph that comes with it) a non-goal. A container is not that. A
consumer that maps a model onto a tool with its own component tree reads the grouping from here.

## 1. API

```go
func (d *Document) Root() *Container
func (c *Container) NewChild(name string) (*Container, error)
func (c *Container) ChildNamed(name string) (*Container, error)      // get or create
func (c *Container) NewUniqueChild(name string) (*Container, error)  // suffixes " 2", " 3", ...
func (c *Container) Path() string                                    // "/a/b"; the root is "/"
func (c *Container) Name() string
func (c *Container) Parent() *Container        // nil for the root
func (c *Container) Children() []*Container    // creation order
func (c *Container) AddSketch(s *sketch.Sketch) error
func (c *Container) AddPlane(p *sketch.Plane) error
func (c *Container) AddBody(b *Body) error
func (c *Container) Sketches() []*sketch.Sketch  // order of AddSketch
func (c *Container) Planes() []*sketch.Plane      // order of AddPlane
func (c *Container) Bodies() []*Body              // live bodies, creation order
func (d *Document) ContainerOf(b *Body) (*Container, error)
func (d *Document) ContainerOfSketch(s *sketch.Sketch) *Container
func (d *Document) VerifyContainers() *ContainerReport
```

- Every document has one root container, named `""`. A body, sketch or plane that was never placed
  belongs to the root.
- `NewChild` takes a non-empty name that is unique among its siblings; anything else is
  `ErrDegenerate`.
- `ChildNamed` returns the child of `c` with that name and adds it when there is none, so calls in a
  loop with one name share one container. It checks the name as `NewChild` does.
- `NewUniqueChild` never fails on a taken name: it appends `" 2"`, `" 3"`, ... and uses the first name
  no sibling has. The base name is checked as `NewChild` checks it. `NewChild` itself keeps refusing a
  duplicate.
- A container belongs to one document for life. There is no delete or rename.
- `AddSketch` and `AddPlane` place the sketch or plane in `c`, and take it out of the container it was
  in. Adding it again to `c` changes nothing. The document keeps the link by the sketch's or plane's
  pointer, so `sketch` needs no change.
- `AddBody` places a live body in `c` the same way; §3 owns its rules.
- `Bodies` lists the live bodies placed directly in `c`, not those of its children. A retired body
  leaves its container with the live set.
- A nil argument is `ErrDegenerate`.

## 2. Where a result lands

Placement is decided where an operation commits, from its inputs, never from a global setting.

| Operation | Result lands in |
|---|---|
| `Union`, `Cut`, `Intersect`, `Combine`, `Hole`, `Fillet`, `Chamfer`, `Shell`, `Draft`, `Thicken`, `Offset`, `Transform`, `Mirrored`, `ScaledAt`, `Patterned`, `PatchEdges`, `Stitch`, `Trim`, `Extend`, `Split`, `Unstitch` | the container of its first (target) body |
| `Copy`, `MirroredCopy`, `PatternCopies` | the container of the source body; every copy lands there |
| `Extrude`, `Revolve`, `Sweep`, `Coil`, `PatchProfile`, `Loft`, `LoftChain` | the container of its sketch (of the first sketch for a loft) |
| `LoftSections`, `LoftFromPoint` | the container of the first section's sketch, or of the point loft's profile sketch |
| `LoftFaces`, `ConeFromFace` | the container of the body the first face belongs to; a `WithConeTangentEnd` cone must share it |

An operation that commits many bodies puts every one of them in the same container.

## 3. Operands from different containers

An operation that reads more than one body or sketch, or a body and a sketch, refuses when they sit in
different containers (a boolean's operands, a loft's sections, a pattern join's target and feature,
an extent that names a body, `SweepPair`'s two bodies): the error is `ErrCrossContainer` (new sentinel, `errors.Is`) naming both
container paths. The check runs with the other operand checks, before any geometry work, and a
refused call leaves the document unchanged.

`c.AddBody(b)` is how a body crosses to another container. It places the live body `b` in `c` and
takes it out of the container it was in. Adding it to the container it is already in changes
nothing. Nothing else changes: the body keeps its handle, coordinates, provenance and measurements,
no producer identity advances, and `Verify` and `Document.Bodies` are unchanged. A nil receiver or
body is `ErrDegenerate`, a body of another document `ErrForeignBody`, and a retired body
`ErrRetiredBody`. A tool that allows a combine only inside one component builds in one container
and adds the finished body to the other.

## 4. Concurrency and state

Container state is document state, guarded by the same lock as the live set. `Body` stays immutable:
its container is not a field of it. `Bodies`, `Remove`, `Verify` and the readers above may run
concurrently with one another; a mutation of the tree (`NewChild`, `AddSketch`, `AddPlane`,
`AddBody`, and so `ChildNamed` and `NewUniqueChild`) is a mutation of the document and not safe to
run concurrently with another.

## 5. What does not change

Geometry, measurements, `Verify`, export and every existing refusal keep their results. A document
that never uses a container behaves as before: every body, sketch and plane is in the root and no
check can fail.

## 6. Layout check

`Document.VerifyContainers` reads the container tree and the live set once, under the document lock,
and changes nothing. It returns a `*ContainerReport`:

| Field or method | Meaning |
|---|---|
| `RootBodies` | Live bodies placed directly in the root, in creation order. |
| `EmptyContainers` | Non-root containers with no live body in them or in any container beneath them, depth first, children in creation order. |
| `Passed()` | True when both lists are empty. |
| `Err()` | Nil when `Passed`; otherwise an error wrapping `ErrContainerLayout` that names each root body by its 1-based place among the live bodies (`#2`) and each empty container by its `Path()`. |

A container that holds bodies only in a container beneath it is not empty: a group of groups is a
valid layout. A parent of empty containers only is empty. Sketches and planes do not make a container
non-empty, since only bodies are the model's content. The report carries no measurement. A document
that never uses a container reports each of its bodies as a root body.

## 7. Tests

| Test | What it shows |
|---|---|
| `apitest.TestContainerChildNamed` | repeated calls return one container; names under different parents differ; empty name is `ErrDegenerate` |
| `apitest.TestContainerNewUniqueChild` | suffixes in order, a taken suffix is skipped, `NewChild` still refuses a duplicate |
| `apitest.TestContainerVerifyContainers` | root bodies, empty and nested containers, the error text, and a clean layout after `AddBody` |


| Test | What it shows |
|---|---|
| `apitest.TestContainerTree` | names, parents, children order, unique sibling names, foreign container |
| `apitest.TestContainerPlacement` | each row of §2 lands where the table says |
| `apitest.TestContainerCrossRefusal` | a boolean, a stitch and an extrude to a body in another container refuse with `ErrCrossContainer`, leaving the document unchanged |
| `apitest.TestContainerAddBody` | `AddBody` changes the container only (volume, bounds and `Verify` unchanged); re-adding to the same container changes nothing; a nil body is `ErrDegenerate` |
