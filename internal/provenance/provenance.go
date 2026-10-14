// Package provenance holds the evaluator's private producer identity and the
// stable role names a builder mints faces and bodies under. The root package's
// FeatureRef is the public form of Ref (docs/evaluator-design.md §3).
package provenance

// Producer is the document-local identity assigned to one successful model
// operation.
type Producer int

// Ref identifies the stable role that created a body, face or edge: the
// producing operation and the role within it.
type Ref struct {
	Producer Producer
	// Role names the entity within the producing feature: "side(i,j)" (loop i,
	// segment j), "capStart", "capEnd", "body".
	Role string
}

// The stable role names the analytic evaluators mint.
const (
	// RoleCapStart is the start cap of a prism sweep.
	RoleCapStart = "capStart"
	// RoleCapEnd is the end cap of a prism sweep.
	RoleCapEnd = "capEnd"
	// RoleBody is the body's own Origin() role.
	RoleBody = "body"
)
