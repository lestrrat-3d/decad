package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// capsAuditFixtureLoft is the point loft of neutralArcSeamSource.
func capsAuditFixtureLoft(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		point := s.CreatePoint(u, v)
		s.Fix(point)
		return point
	}
	center := fixed(0, 0)
	innerRight, innerLeft := fixed(2.2, -0.2), fixed(2.2, 0.2)
	right := []*sketch.Point{fixed(2.8, -0.24), fixed(3.5, -0.3), fixed(4.2, -0.35)}
	left := []*sketch.Point{fixed(2.8, 0.24), fixed(3.5, 0.3), fixed(4.2, 0.35)}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[2], left[2])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(innerRight, right[0])
	s.CreateLine(left[0], innerLeft)
	s.CreateArc(center, innerRight, innerLeft)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	loft, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return loft
}

// capsAuditSplit cuts body at the plane x = x with the fixture's sheet and
// returns the two parts.
func capsAuditSplit(t *testing.T, doc *Document, body *Body, x float64) (*Body, *Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b := s.CreatePoint(x, -20), s.CreatePoint(x, 20)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	sheet, err := Extrude(t.Context(), doc, s, s.Chains()[0], TwoSided{
		One: DistanceSide{D: units.Millimeters(20)},
		Two: DistanceSide{D: units.Millimeters(20)},
	})
	require.NoError(t, err)
	parts, err := Split(t.Context(), body, sheet)
	require.NoError(t, err)
	require.Len(t, parts, 2)
	return parts[0], parts[1]
}

// TestPointLoftAuditsDecideCapsAsFamilies builds the fixture's point loft,
// its first two splits and a LoftFaces over three of their caps, rebuilds
// each body's layout with the production helpers from the body's own mesh,
// and requires the caps entry to decide every cap as a family. This is the
// real producers' layouts reaching the real consumer.
func TestPointLoftAuditsDecideCapsAsFamilies(t *testing.T) {
	t.Parallel()
	doc := New()
	pointLoft := capsAuditFixtureLoft(t, doc)
	loftPayload, ok := pointLoft.n.Payload.(facetedPayload)
	require.True(t, ok)
	n := len(loftPayload.Verts) - 1
	sides := loftPayload.Tris[:n]

	type audited struct {
		name   string
		layout loftmesh.LoftCapLayout
		verts  []r3.Vec
		tris   [][3]int
		proofs int
	}
	var cases []audited
	layout, ok := pointLoftCapLayout(loftPayload.Tris, n)
	require.True(t, ok)
	cases = append(cases, audited{"point loft", layout, loftPayload.Verts, loftPayload.Tris, 1})

	addSlab := func(name string, slab *Body, proofs int) {
		payload, ok := slab.n.Payload.(facetedPayload)
		require.True(t, ok)
		require.NotNil(t, payload.PointPlane)
		require.Equal(t, sides, payload.PointPlane.Base.Tris[:n])
		hasApex := payload.PointPlane.Lower == nil
		upperOffset := n
		if hasApex {
			upperOffset = 1
		}
		layout, ok := loftmesh.PointPlaneSlabCapLayout(payload.PointPlane.Base.Tris[:n], len(payload.Tris), upperOffset, hasApex)
		require.True(t, ok, name)
		cases = append(cases, audited{name, layout, payload.Verts, payload.Tris, proofs})
	}
	inner1, slab0 := capsAuditSplit(t, doc, pointLoft, 7.3)
	addSlab("apex slab 1", inner1, 1)
	addSlab("two-plane slab 0", slab0, 2)
	inner2, slab1 := capsAuditSplit(t, doc, inner1, 6.6)
	addSlab("apex slab 2", inner2, 1)
	addSlab("two-plane slab 1", slab1, 2)

	loft, err := LoftFaces(t.Context(), doc,
		neutralArcSeamCap(t, slab1, refOf(CapStart(slab1))),
		neutralArcSeamCap(t, slab1, refOf(CapEnd(slab1))),
		neutralArcSeamCap(t, slab0, refOf(CapEnd(slab0))))
	require.NoError(t, err)
	facesPayload, ok := loft.n.Payload.(facetedPayload)
	require.True(t, ok)
	facesLayout, ok := loftmesh.LoftFacesCapLayout(sides, 2, len(facesPayload.Tris))
	require.True(t, ok)
	cases = append(cases, audited{"LoftFaces", facesLayout, facesPayload.Verts, facesPayload.Tris, 2})

	for _, c := range cases {
		work, structured, err := loftmesh.LoftCrossingAuditWithCapsWork(t.Context(), c.verts, c.tris, c.layout)
		require.NoError(t, err, c.name)
		require.True(t, structured, c.name)
		require.Equal(t, c.proofs, work.CapProofs, c.name)
		require.NoError(t, loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), c.verts, c.tris), c.name)
		generic, err := loftmesh.LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), c.verts, c.tris,
			loftmesh.LoftAuditShortcuts{BroadPhase: true, Certificates: true, Sweep: true})
		require.NoError(t, err, c.name)
		require.Less(t, work.Candidates, generic.Candidates, c.name)
	}
}
