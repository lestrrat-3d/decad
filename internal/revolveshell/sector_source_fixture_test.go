package revolveshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"

	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/pointloft"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// sectionFixture is one closed polygon in the world XY sketch plane, U along
// world X and V the radius.
func sectionFixture(t *testing.T, outline []sectionrecord.Point2) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(outline))
	for i, point := range outline {
		points[i] = s.CreatePoint(point.U, point.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// revolveFixture builds a full Revolve node of outline about world X with
// the same record, axis resolution and builder a public Revolve uses.
func revolveFixture(t *testing.T, outline []sectionrecord.Point2) *topo.Body {
	t.Helper()
	s, p := sectionFixture(t, outline)
	profile, plane, _, err := sectioncert.RecordProfile(s, p)
	require.NoError(t, err)
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	require.NoError(t, err)
	line, err := revolveaxis.AxisInPlane(revolveaxis.SketchLine{EndU: 1}, frame)
	require.NoError(t, err)
	work := freeform.NewFreeformWork()
	ax, _, err := revolveplan.ResolveAxisSide(t.Context(), profile, line, work)
	require.NoError(t, err)
	full := revolveangle.Sweep{Phi0: revolveangle.Zero(),
		Phi1: revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 1)}}
	body, err := revolveplan.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}},
		revolveplan.Payload{Profile: profile, Frame: frame, Ax: ax, Phi1: 2 * math.Pi, Full: true,
			Den: full, Xform: r3.Identity(), RadialProof: ax.RadialProof}, work)
	require.NoError(t, err)
	return body
}

// sectorFixture is the radial lug of the public sector Union tests: a point
// Loft from the origin to a profile in the X = 10 plane, recorded as a cone
// trim. Its root edge is the arc of radius 4.3. CertifySector reads only the
// source record and its untrimmed base.
func sectorFixture(t *testing.T) *topo.Body {
	t.Helper()
	const rootRadius = 4.3
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rootLow := s.CreatePoint(-rootRadius, -0.4)
	flankLow := s.CreatePoint(-5.5, -0.4)
	tipLow := s.CreatePoint(-6, -0.4)
	tipHigh := s.CreatePoint(-6, 0.4)
	flankHigh := s.CreatePoint(-5.5, 0.4)
	rootHigh := s.CreatePoint(-rootRadius, 0.4)
	center := s.CreatePoint(0, 0)
	for _, point := range []*sketch.Point{rootLow, flankLow, tipLow, tipHigh, flankHigh, rootHigh, center} {
		s.Fix(point)
	}
	s.CreateLine(rootLow, flankLow)
	s.CreateLine(flankLow, tipLow)
	s.CreateArc(center, tipHigh, tipLow)
	s.CreateLine(tipHigh, flankHigh)
	s.CreateLine(flankHigh, rootHigh)
	s.CreateArc(center, rootHigh, rootLow)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	p := profiles[0]
	profile, planeRecord, sketchArea, err := sectioncert.RecordProfile(s, p)
	require.NoError(t, err)
	loftFrame, err := r3.NewFrame(planeRecord.Origin, planeRecord.U, planeRecord.V)
	require.NoError(t, err)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	limit := loftmesh.StationWorkLimit(0, uint64(len(profile.Outer.Segments)))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	work0.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	work1.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	area, err := profile.FalsifyRecordedArea(sketchArea, work0)
	require.NoError(t, err)
	source := &facetedrecord.PointSectionSource{Sketch: s, Profile: p, Revision: s.Revision(),
		Frame: loftFrame, Entities: make([]sketch.Entity, len(p.Outer))}
	for i, edge := range p.Outer {
		source.Entities[i] = edge.Entity
	}
	record := facetedrecord.PointSectionRecord{Profile: profile, Plane: planeRecord, Source: source}
	base, err := pointloft.Build(t.Context(), record, loftFrame, area, work0, work1, 2)
	require.NoError(t, err)
	return &topo.Body{Payload: facetedrecord.Payload{Xform: r3.Identity(),
		PointCone: &facetedrecord.PointConeTrimRecord{Base: base,
			Lower: &facetedrecord.PointConeLimit{Axis: coneaxis.X()}, Upper: &facetedrecord.PointConeLimit{Axis: coneaxis.X()}}}}
}

func sectorPayload(sector *topo.Body) facetedrecord.Payload {
	return sector.Payload.(facetedrecord.Payload)
}
