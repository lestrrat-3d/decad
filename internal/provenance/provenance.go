// Package provenance holds the evaluator's provenance values: the
// document-local identity of one successful operation and the stable role it
// minted (agent-docs/evaluator-design.md §3). The root FeatureRef is their public
// form; the topology nodes hold these.
package provenance

// Producer is the document-local identity assigned to one successful model
// operation. It supports provenance while keeping the model's execution
// bookkeeping private.
type Producer int

// Ref identifies the stable role that created a body, face, or edge: the
// producing operation and the role within it ("side(i,j)", "capStart",
// "capEnd", "body").
type Ref struct {
	Producer Producer
	Role     string
}

// The stable role names every analytic evaluator mints (evaluator §3): the
// body's own Origin role, and the two ends of a prism sweep or of a revolve's
// partial turn.
const (
	RoleBody     = "body"
	RoleCapStart = "capStart"
	RoleCapEnd   = "capEnd"
)
