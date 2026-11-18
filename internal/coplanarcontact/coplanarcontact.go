// Package coplanarcontact carries the switch that turns on exact classification
// of coplanar facet contacts in the mesh boolean
// (agent-docs/coplanar-contact-design.md). The choice rides on the context an
// operation already passes down, as rimsplit's does.
package coplanarcontact

import "context"

type key struct{}

// With returns ctx carrying the choice: on classifies the coplanar contacts of
// exact planar faces in every mesh boolean the context reaches.
func With(ctx context.Context, on bool) context.Context {
	return context.WithValue(ctx, key{}, on)
}

// Of reports whether ctx carries the choice to classify, and false when it
// carries none.
func Of(ctx context.Context) bool {
	on, _ := ctx.Value(key{}).(bool)
	return on
}
