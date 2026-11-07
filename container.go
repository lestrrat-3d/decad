package decad

import (
	"fmt"
	"slices"
	"strings"

	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/decad/internal/topo"
)

// ErrCrossContainer is returned when one operation reads bodies or sketches that
// sit in different containers (agent-docs/container-design.md §3). MoveBody is the
// one way across.
var ErrCrossContainer = fmt.Errorf("decad: operands sit in different containers")

// ErrContainerLayout is the error ContainerReport.Err returns when a body sits
// directly in the root or a non-root container holds no bodies
// (agent-docs/container-design.md §6).
var ErrContainerLayout = fmt.Errorf("decad: container layout is not clean")

// Container is a named group inside one Document. It holds sketches, planes and
// bodies and nests inside other containers. It is bookkeeping only: it has no
// transform, instances nothing and changes no geometry (agent-docs/container-design.md).
//
// A Container belongs to one document for life. Its readers are safe to call
// concurrently with Bodies, Remove and Verify; NewChild, ChildNamed, NewUniqueChild,
// AddSketch, AddPlane and MoveBody mutate the document and are not safe to run concurrently with another
// mutation.
type Container struct {
	doc      *Document
	name     string
	parent   *Container
	children []*Container
	sketches []*sketch.Sketch
	planes   []*sketch.Plane
}

// containerState is a document's container tree and the placement of its
// members. Every field is guarded by the document node's LiveMu.
type containerState struct {
	root     *Container
	ofBody   map[*topo.Body]*Container
	ofSketch map[*sketch.Sketch]*Container
	ofPlane  map[*sketch.Plane]*Container
}

// rootLocked returns the root container, minting it on first use. The caller
// holds LiveMu for writing.
func (d *Document) rootLocked() *Container {
	if d.groups.root == nil {
		d.groups.root = &Container{doc: d}
	}
	return d.groups.root
}

// Root returns the document's root container, named "". A body, sketch or plane
// never placed elsewhere belongs to it.
func (d *Document) Root() *Container {
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	return d.rootLocked()
}

// NewChild adds a container named name under c. The name must be non-empty and
// unique among c's children (ErrDegenerate); a container of another document is
// ErrForeignBody.
func (c *Container) NewChild(name string) (*Container, error) {
	if c == nil {
		return nil, fmt.Errorf(`%w: a nil container holds no child`, ErrDegenerate)
	}
	if name == "" {
		return nil, fmt.Errorf(`%w: a container needs a non-empty name`, ErrDegenerate)
	}
	d := c.doc
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if c.childLocked(name) != nil {
		return nil, fmt.Errorf(`%w: container %q already has a child named %q`, ErrDegenerate, c.pathLocked(), name)
	}
	child := &Container{doc: d, name: name, parent: c}
	c.children = append(c.children, child)
	return child, nil
}

// ChildNamed returns the child of c named name, adding it when c has none. Calls
// with one name on one container return the same container. The name is checked as
// NewChild checks it (ErrDegenerate).
func (c *Container) ChildNamed(name string) (*Container, error) {
	if c == nil {
		return nil, fmt.Errorf(`%w: a nil container holds no child`, ErrDegenerate)
	}
	if name == "" {
		return nil, fmt.Errorf(`%w: a container needs a non-empty name`, ErrDegenerate)
	}
	d := c.doc
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if child := c.childLocked(name); child != nil {
		return child, nil
	}
	child := &Container{doc: d, name: name, parent: c}
	c.children = append(c.children, child)
	return child, nil
}

// NewUniqueChild adds a container under c named name, or, when a child already has
// that name, name followed by " 2", " 3" and so on: the first free one. The name is
// checked as NewChild checks it (ErrDegenerate).
func (c *Container) NewUniqueChild(name string) (*Container, error) {
	if c == nil {
		return nil, fmt.Errorf(`%w: a nil container holds no child`, ErrDegenerate)
	}
	if name == "" {
		return nil, fmt.Errorf(`%w: a container needs a non-empty name`, ErrDegenerate)
	}
	d := c.doc
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	unique := name
	for n := 2; c.childLocked(unique) != nil; n++ {
		unique = fmt.Sprintf("%s %d", name, n)
	}
	child := &Container{doc: d, name: unique, parent: c}
	c.children = append(c.children, child)
	return child, nil
}

// childLocked returns the child of c named name, or nil. The caller holds LiveMu.
func (c *Container) childLocked(name string) *Container {
	for _, child := range c.children {
		if child.name == name {
			return child
		}
	}
	return nil
}

// Path spells c as its names from the root, joined by "/"; the root is "/".
func (c *Container) Path() string { return c.pathLocked() }

// Name returns the container's name; the root's is "".
func (c *Container) Name() string { return c.name }

// Parent returns the container c nests in, or nil for the root.
func (c *Container) Parent() *Container { return c.parent }

// Children lists c's child containers in creation order. The slice is a copy.
func (c *Container) Children() []*Container {
	c.doc.n.LiveMu.RLock()
	defer c.doc.n.LiveMu.RUnlock()
	return slices.Clone(c.children)
}

// AddSketch places s in c and takes it out of the container it was in. Adding it
// again to c changes nothing. A sketch never added belongs to the root. A nil s is
// ErrDegenerate.
func (c *Container) AddSketch(s *sketch.Sketch) error {
	if c == nil || s == nil {
		return fmt.Errorf(`%w: a nil container or sketch names nothing to place`, ErrDegenerate)
	}
	d := c.doc
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if from := d.sketchContainerLocked(s); from != c {
		from.sketches = slices.DeleteFunc(from.sketches, func(x *sketch.Sketch) bool { return x == s })
		c.sketches = append(c.sketches, s)
	}
	if d.groups.ofSketch == nil {
		d.groups.ofSketch = map[*sketch.Sketch]*Container{}
	}
	d.groups.ofSketch[s] = c
	return nil
}

// AddPlane places p in c and takes it out of the container it was in, as
// AddSketch does for a sketch.
func (c *Container) AddPlane(p *sketch.Plane) error {
	if c == nil || p == nil {
		return fmt.Errorf(`%w: a nil container or plane names nothing to place`, ErrDegenerate)
	}
	d := c.doc
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	from := d.rootLocked()
	if at, ok := d.groups.ofPlane[p]; ok {
		from = at
	}
	if from != c {
		from.planes = slices.DeleteFunc(from.planes, func(x *sketch.Plane) bool { return x == p })
		c.planes = append(c.planes, p)
	}
	if d.groups.ofPlane == nil {
		d.groups.ofPlane = map[*sketch.Plane]*Container{}
	}
	d.groups.ofPlane[p] = c
	return nil
}

// Sketches lists the sketches added to c, in the order they were added.
func (c *Container) Sketches() []*sketch.Sketch {
	c.doc.n.LiveMu.RLock()
	defer c.doc.n.LiveMu.RUnlock()
	return slices.Clone(c.sketches)
}

// Planes lists the planes added to c, in the order they were added.
func (c *Container) Planes() []*sketch.Plane {
	c.doc.n.LiveMu.RLock()
	defer c.doc.n.LiveMu.RUnlock()
	return slices.Clone(c.planes)
}

// Bodies lists the live bodies placed directly in c, in creation order. A body
// of a child container is not listed. A retired body has left its container.
func (c *Container) Bodies() []*Body {
	d := c.doc
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	var out []*topo.Body
	for _, b := range d.n.LiveSet {
		if d.bodyContainerLocked(b) == c {
			out = append(out, b)
		}
	}
	return bodyHandles(out)
}

// ContainerOf returns the container holding the live body b: the root until
// MoveBody or an operation's placement rule says otherwise. A foreign body is
// ErrForeignBody and a retired one ErrRetiredBody.
func (d *Document) ContainerOf(b *Body) (*Container, error) {
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	return d.bodyContainerLocked(bodyNode(b)), nil
}

// ContainerOfSketch returns the container holding s: the root until AddSketch.
func (d *Document) ContainerOfSketch(s *sketch.Sketch) *Container {
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	return d.sketchContainerLocked(s)
}

// MoveBody puts the live body b in to, and changes nothing else: b keeps its
// handle, coordinates, provenance and measurements, and no producer identity
// advances. A foreign body or container is ErrForeignBody, a retired body
// ErrRetiredBody, and a nil argument ErrDegenerate.
func (d *Document) MoveBody(b *Body, to *Container) error {
	if to == nil {
		return fmt.Errorf(`%w: a nil container holds no body`, ErrDegenerate)
	}
	if to.doc != d {
		return ErrForeignBody
	}
	if err := d.requireLive(b); err != nil {
		return err
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	d.placeLocked(bodyNode(b), to)
	return nil
}

func (d *Document) bodyContainerLocked(b *topo.Body) *Container {
	if c, ok := d.groups.ofBody[b]; ok {
		return c
	}
	return d.rootLocked()
}

func (d *Document) sketchContainerLocked(s *sketch.Sketch) *Container {
	if c, ok := d.groups.ofSketch[s]; ok {
		return c
	}
	return d.rootLocked()
}

// placeLocked records that b lives in c. The caller holds LiveMu for writing.
func (d *Document) placeLocked(b *topo.Body, c *Container) {
	if c == d.rootLocked() {
		delete(d.groups.ofBody, b)
		return
	}
	if d.groups.ofBody == nil {
		d.groups.ofBody = map[*topo.Body]*Container{}
	}
	d.groups.ofBody[b] = c
}

// pathLocked spells c as its names from the root, joined by "/"; the root is "/".
func (c *Container) pathLocked() string {
	var names []string
	for at := c; at != nil && at.parent != nil; at = at.parent {
		names = append(names, at.name)
	}
	slices.Reverse(names)
	return "/" + strings.Join(names, "/")
}

// extentBodies lists the bodies an extent names: the target of a ToFace, alone or as
// a side of a TwoSided.
func extentBodies(e Extent) []*Body {
	var out []*Body
	switch e := e.(type) {
	case ToFace:
		out = append(out, e.Body)
	case *ToFace:
		if e != nil {
			out = append(out, e.Body)
		}
	case TwoSided:
		for _, side := range []SideExtent{e.One, e.Two} {
			if t, ok := side.(ToFace); ok {
				out = append(out, t.Body)
			}
		}
	}
	return out
}

// bodyContainer returns the container of body b, whichever state b is in: a retired
// or unplaced body reads as the root.
func (d *Document) bodyContainer(b *Body) *Container {
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	return d.bodyContainerLocked(bodyNode(b))
}

// faceBodies returns the bodies the faces belong to, in order.
func faceBodies(faces []*Face) []*Body {
	out := make([]*Body, 0, len(faces))
	for _, f := range faces {
		if f != nil {
			out = append(out, bodyHandle(f.n.Body))
		}
	}
	return out
}

// resultIn makes every commit that names no container place its result in c, until
// the returned function runs. It serves an operation whose commit sits several
// calls below the code that knows the container; documents are not safe for
// concurrent mutation, so one holder at a time is the contract.
func (d *Document) resultIn(c *Container) func() {
	d.n.LiveMu.Lock()
	prev := d.resultC
	d.resultC = c
	d.n.LiveMu.Unlock()
	return func() {
		d.n.LiveMu.Lock()
		d.resultC = prev
		d.n.LiveMu.Unlock()
	}
}

// requireSameContainer refuses with ErrCrossContainer when the bodies and sketches
// an operation reads do not all sit in one container. It runs with the operand
// checks, before any geometry work. Nil members are skipped.
func (d *Document) requireSameContainer(bodies []*Body, sketches ...*sketch.Sketch) error {
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	var first *Container
	check := func(c *Container) error {
		if first == nil {
			first = c
			return nil
		}
		if c != first {
			return fmt.Errorf(`%w: %q and %q`, ErrCrossContainer, first.pathLocked(), c.pathLocked())
		}
		return nil
	}
	for _, b := range bodies {
		if b == nil || b.n.Doc != documentNode(d) {
			continue
		}
		if err := check(d.bodyContainerLocked(bodyNode(b))); err != nil {
			return err
		}
	}
	for _, s := range sketches {
		if s == nil {
			continue
		}
		if err := check(d.sketchContainerLocked(s)); err != nil {
			return err
		}
	}
	return nil
}

// ContainerReport is what VerifyContainers found. Both lists are in a fixed order:
// RootBodies in creation order and EmptyContainers in depth-first order of the tree,
// children in creation order.
type ContainerReport struct {
	// RootBodies are the live bodies placed directly in the root container.
	RootBodies []*Body
	// EmptyContainers are the non-root containers with no live body in them or in
	// any container beneath them.
	EmptyContainers []*Container

	// rootIndex numbers each RootBodies member by its place in the live set, from 1.
	rootIndex []int
}

// Passed reports whether the layout is clean: no body in the root and no empty
// container.
func (r *ContainerReport) Passed() bool {
	return len(r.RootBodies) == 0 && len(r.EmptyContainers) == 0
}

// Err returns nil when Passed, and otherwise an error wrapping ErrContainerLayout.
// It names each root body by its 1-based place among the live bodies and each empty
// container by its path.
func (r *ContainerReport) Err() error {
	if r.Passed() {
		return nil
	}
	var parts []string
	if len(r.rootIndex) > 0 {
		places := make([]string, len(r.rootIndex))
		for i, n := range r.rootIndex {
			places[i] = fmt.Sprintf("#%d", n)
		}
		parts = append(parts, fmt.Sprintf("live bodies %s sit in the root container", strings.Join(places, ", ")))
	}
	if len(r.EmptyContainers) > 0 {
		paths := make([]string, len(r.EmptyContainers))
		for i, c := range r.EmptyContainers {
			paths[i] = fmt.Sprintf("%q", c.Path())
		}
		parts = append(parts, fmt.Sprintf("containers %s hold no bodies", strings.Join(paths, ", ")))
	}
	return fmt.Errorf("%w: %s", ErrContainerLayout, strings.Join(parts, "; "))
}

// VerifyContainers checks the document's container layout without touching
// geometry: every live body should sit in a named container, and every non-root
// container should hold a body, directly or in a container beneath it. A document
// that never uses a container reports every body as a root body.
func (d *Document) VerifyContainers() *ContainerReport {
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	root := d.rootLocked()
	rep := &ContainerReport{}
	held := map[*Container]struct{}{}
	for i, b := range d.n.LiveSet {
		c := d.bodyContainerLocked(b)
		if c == root {
			rep.RootBodies = append(rep.RootBodies, bodyHandle(b))
			rep.rootIndex = append(rep.rootIndex, i+1)
			continue
		}
		for at := c; at != nil; at = at.parent {
			held[at] = struct{}{}
		}
	}
	var walk func(c *Container)
	walk = func(c *Container) {
		for _, child := range c.children {
			if _, ok := held[child]; !ok {
				rep.EmptyContainers = append(rep.EmptyContainers, child)
			}
			walk(child)
		}
	}
	walk(root)
	return rep
}
