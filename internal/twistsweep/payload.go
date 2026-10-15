package twistsweep

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the twisted sweep's payloads: the records Eval evaluates over
// topology nodes and a placement re-evaluates. Both implement topo.Payload
// (docs/evaluator-design.md §8, docs/sweep-design.md §17).

// Payload keeps the true rigidly rotating section separate from the Loft
// polyhedron used as its held boundary. In particular, consumers must not
// mistake the held facets for the exact shape of the sweep.
type Payload struct {
	Profile    profileRecord
	Plane      planeRecord
	Path       *sweepinput.Path
	Angle      units.Value
	RecordArea float64
	Xform      r3.Transform
	Held       loftrecord.Payload
	Mesh       *topo.Mesh
	// SourceRoles maps Loft's private triangle roles to the merged live
	// Sweep faces. Both wall triangles of one profile edge share one face.
	SourceRoles map[string]*topo.Face
	Delta       float64
	AreaSlack   float64
	VolSymDiff  float64
}

// Transform is the accumulated rigid placement.
func (p Payload) Transform() r3.Transform { return p.Xform }

// Placed re-evaluates the same record under the composed motion xform.
func (p Payload) Placed(ctx context.Context, env topo.Env, xform r3.Transform) (*topo.Body, error) {
	return Eval(ctx, env, p.Profile, p.Plane, p.Path, p.Angle, p.RecordArea, xform)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (docs/evaluator-design.md §8): the root type this payload replaced.
func (Payload) SurveyClass() string { return "twistedSweepPayload" }

// CompositePayload is the record of a twist along a line and a cardinal
// quarter arc: the true curved-path section, held by one line Loft and eight
// arc Lofts sewn into Mesh.
type CompositePayload struct {
	Profile    profileRecord
	Plane      planeRecord
	Path       *sweepinput.Path
	Angle      units.Value
	RecordArea float64
	Xform      r3.Transform
	Mesh       *topo.Mesh
	Delta      float64
}

// Transform is the accumulated rigid placement.
func (p CompositePayload) Transform() r3.Transform { return p.Xform }

// Placed re-evaluates the same record under the composed motion xform.
func (p CompositePayload) Placed(ctx context.Context, env topo.Env, xform r3.Transform) (*topo.Body, error) {
	return evalComposite(ctx, env, p.Profile, p.Plane, p.Path, p.Angle, p.RecordArea, xform)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (docs/evaluator-design.md §8): the root type this payload replaced.
func (CompositePayload) SurveyClass() string { return "compositeTwistedSweepPayload" }
