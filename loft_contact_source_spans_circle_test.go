package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftCircleSourceCapsRealDiscRefusesWithoutBoundaryProof(t *testing.T) {
	world := sketch.NewWorld()
	doc := New()
	upper, err := world.CreateOffsetPlane(world.XY(), 10)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, radius := range [2]float64{5, 5.5} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		right := s.CreatePoint(radius, 0)
		top := s.CreatePoint(0, radius)
		s.CreateLine(center, right)
		s.CreateArc(center, right, top)
		s.CreateLine(top, center)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		found := s.Profiles()
		require.Equal(t, 1, len(found))
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)

	discSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := discSketch.CreatePoint(0, 0)
	discSketch.Fix(center)
	discSketch.CreateCircle(center, 4)
	_, err = discSketch.Solve(t.Context())
	require.NoError(t, err)
	discProfiles := discSketch.Profiles()
	require.Equal(t, 1, len(discProfiles))
	require.True(t, discProfiles[0].Valid)
	prism, err := Extrude(t.Context(), doc, discSketch, discProfiles[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.True(t, prism.n.Doc == loft.n.Doc)
	require.NotEqual(t, prism.n.Origin.Producer, loft.n.Origin.Producer)
	prismSource := prism.n.Payload.(prismPayload).Profile.Outer.Segments
	require.Equal(t, 1, len(prismSource))
	circle, isCircle := prismSource[0].(sectionrecord.CircleSeg)
	require.True(t, isCircle)
	radius, err := circle.Radius.In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, float64(4), radius)

	spans, accepted, err := certifyLoftSourceSpanCaps(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		prism, loft)
	require.NoError(t, err)
	require.True(t, accepted)
	require.Positive(t, spans.visits)
	require.Less(t, spans.visits, uint64(4096))
	for cap := range 2 {
		require.Equal(t, 1, len(spans.prismCaps[cap]))
		span := spans.prismCaps[cap][0]
		require.True(t, span.body == prism)
		require.Equal(t, 0, span.segment)
		role := provenance.RoleCapStart
		if cap == 1 {
			role = provenance.RoleCapEnd
		}
		require.Equal(t, role, span.capFace.Role)
		require.Equal(t, prism.n.Origin.Producer, span.capFace.Producer)
		require.Equal(t, 0, len(span.wallFaces))
		got, ok := span.curve.(sectionrecord.CircleSeg)
		require.True(t, ok)
		require.Equal(t, circle, got)
		require.Equal(t, circle.TStart, span.tStart)
		require.Equal(t, circle.TEnd, span.tEnd)
		require.Equal(t, circle.TStart > circle.TEnd, span.reversed)
	}
	for _, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, admitted, sourceErr := certifyLoftSourcePartition(t.Context(), pair[0], pair[1],
			proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, sourceErr)
		require.False(t, admitted)

		steps := 0
		stopAtReader := &proofbound.WorkBudget{
			StepFn: func() error {
				steps++
				return proofbound.ErrWallWorkBudget
			},
			ErrFn: func() error { return nil },
		}
		_, admitted, sourceErr = certifyLoftSourcePartition(t.Context(), pair[0], pair[1], stopAtReader)
		require.ErrorIs(t, sourceErr, proofbound.ErrWallWorkBudget)
		require.False(t, admitted)
		require.Equal(t, 1, steps)
	}
	for _, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
		result, unionErr := Union(t.Context(), pair[0], pair[1])
		require.ErrorIs(t, unionErr, ErrUnsupported)
		require.True(t, result == nil)
	}
}

func TestLoftCircleSourceSpanRefusesInvalidRecord(t *testing.T) {
	valid := sectionrecord.CircleSeg{
		Radius: units.Millimeters(4), CCW: true, TStart: 0, TEnd: 1,
	}
	for _, tc := range []struct {
		name   string
		change func(*sectionrecord.CircleSeg)
	}{
		{"zero radius", func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(0) }},
		{"negative radius", func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(-4) }},
		{"wrong unit", func(c *sectionrecord.CircleSeg) { c.Radius = units.Radians(4) }},
		{"infinite radius", func(c *sectionrecord.CircleSeg) { c.Radius = units.Millimeters(math.Inf(1)) }},
		{"partial range", func(c *sectionrecord.CircleSeg) { c.TEnd = 0.75 }},
		{"wrong winding", func(c *sectionrecord.CircleSeg) { c.CCW = false }},
		{"nonfinite center", func(c *sectionrecord.CircleSeg) { c.Center.U = math.NaN() }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			circle := valid
			tc.change(&circle)
			_, accepted := loftSourceCapSpanOf(nil, provenance.RoleCapStart, 0, circle)
			require.False(t, accepted)
		})
	}
}
