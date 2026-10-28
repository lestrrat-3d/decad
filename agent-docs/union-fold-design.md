# Union fold design

`Union` of three or more bodies takes the certified routes of `boolean_many.go` as a set. When none of them takes the set,
it joins the bodies two at a time, left to right, instead of refusing. The fold is `boolean_fold.go`.

## 1. Rules

| Decision | Rule |
|---|---|
| U1 | The fold runs only when `performUnionMany` refuses with `ErrUnsupported` and the refusal is the one that says no certified route takes the set. Every other refusal, and every gate that runs before it (a repeated body, a retired body, a foreign body, a sheet), is returned as it was. |
| U2 | The bodies are joined in the order given: the first with the second, that result with the third, and so on. Each step is the two-body `Union`, so it takes that `Union`'s routes, its bounds and its refusals, and the result is a mesh Boolean's where no analytic route applies. |
| U3 | The chain is run on copies first, each retired when read, so a step that refuses leaves every body live and the document as it was, and the refusal returned is the many-body one. Only a chain that completes on the copies is run on the bodies. The copies leave history entries. |
| U4 | The result retires the bodies and every intermediate, and the history holds one `OperationUnion` for each step. |
| U5 | The volume of the result is that of the union to the exactness of the mesh steps, whatever the order. The tests compute it independently by integrating the union's cross-section. |
| U6 | `PatternJoin` takes the fold too: when its one Union of the target, the feature and the instances is refused the many-body way, the instances are built again as registered copies (`PatternCopies`) and `Union(target, feature, copies...)` folds them, the copies removed if the fold refuses and the first refusal returned. The tests join three overlapping hexagonal pins, as tall as a slanted mesh plate and flush with both its faces, to the plate (7500 within a bound under 0.1), and, under `ExactOnly`, keep the refusal with the plate and the pin live. |
| U7 | A fold step meets contacts whose normal is the zero vector, from a sliver facet of the earlier result that is too thin to normalise. The shifted retry gives such a contact no direction to open along (`shiftOpens` skips it) and reaches its slack along the shift's own length (`shiftNeed`), where before it left no direction at all and ended the retry. |

## 2. Not covered

- `PatternJoin` and `PathPatternJoin`'s instances take `unionManyBodies` directly and are not folded: a set that no certified
  route takes still refuses in `PatternJoin`.
- A chain whose step refuses on its own geometry, such as coarse held meshes: the fold returns the many-body refusal.
