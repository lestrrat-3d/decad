package sweepmitre

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload is the evaluator's record of a mitred polyline sweep
// (agent-docs/sweep-design.md §16): the authenticated profile and its plane, the
// path, one scale factor per span and the accumulated placement — what a
// placement re-runs §16.3 from — plus the build that record produced. Build
// evaluates it over topology nodes and it implements topo.Payload
// (agent-docs/evaluator-design.md §8).
//
// Exact holds every vertex as the rational the construction denotes, placed
// exactly under Xform; Verts is each rounded once to the nearest float,
// VertexBound is each vertex's own 3D gap between the two, rounded up, and
// Delta is the largest of them. Tris is the globally oriented
// held triangle set, walls first in (span, loop, segment) order, then capStart
// and capEnd; TriFace names each triangle's face as an index into FaceRoles.
type Payload struct {
	Profile momentinput.Profile
	Plane   sectionrecord.PlaneRecord
	Path    *sweepinput.Path
	Factors []float64
	Xform   r3.Transform

	Exact       []sweeparc.RatVec
	Verts       []r3.Vec
	VertexBound []float64
	Tris        [][3]int
	TriFace     []int
	FaceRoles   []string
	Delta       float64
	// These sums use the unplaced sections and canonical triangle winding.
	// Placements reuse them before deciding the new shell orientation.
	LocalVol6    *big.Rat
	LocalMoments [3]*big.Rat
}

// Transform is the accumulated rigid placement.
func (mp Payload) Transform() r3.Transform { return mp.Xform }

// AxialDelta is every held vertex's displacement; a ToFace stop against
// either cap inherits it.
func (mp Payload) AxialDelta() float64 { return mp.Delta }

// Placed re-runs §16.3 from the record under the composed motion (Table DM
// row DM7): the construction rebuilds the unplaced rationals, applies the
// motion to them exactly, rounds once, re-decides the shell orientation and
// re-runs the crossing audit, so Delta never accumulates across placements.
func (mp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	return Build(ctx, env, Payload{
		Profile:      mp.Profile,
		Plane:        mp.Plane,
		Path:         mp.Path,
		Factors:      mp.Factors,
		Xform:        composed,
		LocalVol6:    mp.LocalVol6,
		LocalMoments: mp.LocalMoments,
	})
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "mitredSweepPayload" }
