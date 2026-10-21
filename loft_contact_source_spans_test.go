package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftdirect"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftCurvedSourceCapsRealIndependentWedgeRefusesWithoutBoundaryProof(t *testing.T) {
	world := sketch.NewWorld()
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
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	lp := loft.n.Payload.(loftPayload)
	for _, loop := range []sectionrecord.LoopRecord{lp.Profile0.Outer, lp.Profile1.Outer} {
		require.Equal(t, 3, len(loop.Segments))
		var lines, arcs int
		for _, segment := range loop.Segments {
			switch segment.(type) {
			case sectionrecord.LineSeg:
				lines++
			case sectionrecord.ArcSeg:
				arcs++
			}
		}
		require.Equal(t, 2, lines)
		require.Equal(t, 1, arcs)
	}

	prismSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.Zero(t, prism.n.Origin.Producer)
	require.Zero(t, loft.n.Origin.Producer)
	require.False(t, prism.n.Doc == loft.n.Doc)
	certificate, certified, certErr := loftdirect.CertifyLoftSourceSpanCaps(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		bodyNode(prism), bodyNode(loft))
	require.NoError(t, certErr)
	require.True(t, certified)
	require.True(t, certificate.Prism == bodyNode(prism) && certificate.Loft == bodyNode(loft))
	require.Equal(t, [3]string{"0", "10", "12"}, [3]string{
		certificate.Levels[0].RatString(), certificate.Levels[1].RatString(),
		certificate.Levels[2].RatString(),
	})
	require.Positive(t, certificate.Visits)
	require.Less(t, certificate.Visits, uint64(4096))
	for cap := range 2 {
		require.Equal(t, 4, len(certificate.PrismCaps[cap]))
		require.Equal(t, 3, len(certificate.LoftCaps[cap]))
		var arcsWithWalls int
		for _, span := range certificate.LoftCaps[cap] {
			require.True(t, span.Body == bodyNode(loft))
			require.Equal(t, float64(0), span.TStart)
			require.Equal(t, float64(1), span.TEnd)
			if _, arc := span.Curve.(sectionrecord.ArcSeg); arc {
				require.Positive(t, len(span.WallFaces))
				arcsWithWalls++
			}
		}
		require.Equal(t, 1, arcsWithWalls)
	}
	ordered, certifiedOrder, orderErr := loftdirect.CertifyLoftCapSourceOrder(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		certificate)
	require.NoError(t, orderErr)
	require.True(t, certifiedOrder)
	require.Positive(t, ordered.Visits)
	for cap := range 2 {
		require.NotEmpty(t, ordered.Caps[cap])
		var sharedCells, cutArcs int
		for _, cell := range ordered.Caps[cap] {
			require.NotEmpty(t, cell.Edges)
			if cell.Prism && cell.Loft {
				sharedCells++
			}
			for _, edge := range cell.Edges {
				require.True(t, edge.Span.Body == bodyNode(prism) || edge.Span.Body == bodyNode(loft))
				require.Less(t, edge.TStart, edge.TEnd)
				require.Equal(t, edge.Span.Body.Origin.Producer,
					edge.Span.CapFace.Producer)
				if _, arc := edge.Span.Curve.(sectionrecord.ArcSeg); arc && edge.Partial {
					require.True(t, edge.Span.Body == bodyNode(loft))
					cutArcs++
				}
			}
		}
		require.Positive(t, sharedCells)
		require.Positive(t, cutArcs)
	}
	_, certifiedOrder, orderErr = loftdirect.CertifyLoftCapSourceOrder(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context())),
		certificate)
	require.ErrorIs(t, orderErr, proofbound.ErrWallWorkBudget)
	require.False(t, certifiedOrder)
	canceled, cancel := context.WithCancel(t.Context())
	steps := 0
	operation := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 30 {
				cancel()
			}
			return canceled.Err()
		},
		ErrFn: canceled.Err,
	}
	_, certifiedOrder, orderErr = loftdirect.CertifyLoftCapSourceOrder(canceled,
		proofbound.NewWallWorkBudgetWithOperation(4096, operation), certificate)
	require.ErrorIs(t, orderErr, context.Canceled)
	require.False(t, certifiedOrder)
	require.Equal(t, 30, steps)
	t.Run("over_cap_arcs", func(t *testing.T) {
		const arcs = 17
		arcWorld := sketch.NewWorld()
		arcUpper, upperErr := arcWorld.CreateOffsetPlane(arcWorld.XY(), 10)
		require.NoError(t, upperErr)
		var arcSections [2]*sketch.Sketch
		var arcProfiles [2]*sketch.Profile
		for cap, radius := range [2]float64{5, 5.5} {
			plane := arcWorld.XY()
			if cap == 1 {
				plane = arcUpper
			}
			s, createErr := arcWorld.CreateSketch(plane)
			require.NoError(t, createErr)
			center := s.CreatePoint(0, 0)
			s.Fix(center)
			points := make([]*sketch.Point, arcs+1)
			for i := range points {
				angle := math.Pi * float64(i) / (2 * arcs)
				points[i] = s.CreatePoint(radius*math.Cos(angle), radius*math.Sin(angle))
			}
			s.CreateLine(center, points[0])
			for i := range arcs {
				s.CreateArc(center, points[i], points[i+1])
			}
			s.CreateLine(points[arcs], center)
			_, createErr = s.Solve(t.Context())
			require.NoError(t, createErr)
			found := s.Profiles()
			require.Len(t, found, 1)
			require.True(t, found[0].Valid)
			arcSections[cap], arcProfiles[cap] = s, found[0]
		}
		arcLoft, loftErr := Loft(t.Context(), New(), arcSections[0], arcProfiles[0],
			arcSections[1], arcProfiles[1])
		require.NoError(t, loftErr)
		arcSource, sourceOK, sourceErr := loftdirect.CertifyLoftSourceSpanCaps(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
			bodyNode(prism), bodyNode(arcLoft))
		require.NoError(t, sourceErr)
		require.True(t, sourceOK)
		require.Len(t, arcSource.LoftCaps[0], arcs+2)
		arcCount := 0
		for _, span := range arcSource.LoftCaps[0] {
			if _, isArc := span.Curve.(sectionrecord.ArcSeg); isArc {
				arcCount++
			}
		}
		require.Equal(t, arcs, arcCount)
		conservativeSegments := len(arcSource.PrismCaps[0]) +
			len(arcSource.LoftCaps[0]) - arcCount + 256*arcCount
		require.Greater(t, conservativeSegments, prismcells.MaxArrangementSegments)
		steps := 0
		operation := &proofbound.WorkBudget{
			StepFn: func() error { steps++; return nil },
			ErrFn:  t.Context().Err,
		}
		_, admitted, capErr := loftdirect.CertifyLoftCapSourceOrder(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(4096, operation), arcSource)
		require.NoError(t, capErr)
		require.False(t, admitted)
		require.Positive(t, steps)
		require.Less(t, steps,
			len(arcSource.PrismCaps[0])+len(arcSource.LoftCaps[0]))
	})
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			source, admitted, sourceErr := loftdirect.CertifyLoftSourcePartition(t.Context(),
				bodyNode(tc.first), bodyNode(tc.second), proofbound.NewWorkBudget(t.Context()), false)
			require.NoError(t, sourceErr)
			require.False(t, admitted)
			require.Zero(t, source.SourceVisits)

			boundary, accepted, boundaryErr := loftdirect.CertifyLoftOutputBoundary(t.Context(),
				bodyNode(tc.first), bodyNode(tc.second), proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, boundaryErr)
			require.False(t, accepted)
			require.Zero(t, len(boundary.Shell.Vertices))
		})
	}
}

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

	spans, accepted, err := loftdirect.CertifyLoftSourceSpanCaps(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		bodyNode(prism), bodyNode(loft))
	require.NoError(t, err)
	require.True(t, accepted)
	require.Positive(t, spans.Visits)
	require.Less(t, spans.Visits, uint64(4096))
	for cap := range 2 {
		require.Equal(t, 1, len(spans.PrismCaps[cap]))
		span := spans.PrismCaps[cap][0]
		require.True(t, span.Body == bodyNode(prism))
		require.Equal(t, 0, span.Segment)
		role := provenance.RoleCapStart
		if cap == 1 {
			role = provenance.RoleCapEnd
		}
		require.Equal(t, role, span.CapFace.Role)
		require.Equal(t, prism.n.Origin.Producer, span.CapFace.Producer)
		require.Equal(t, 0, len(span.WallFaces))
		got, ok := span.Curve.(sectionrecord.CircleSeg)
		require.True(t, ok)
		require.Equal(t, circle, got)
		require.Equal(t, circle.TStart, span.TStart)
		require.Equal(t, circle.TEnd, span.TEnd)
		require.Equal(t, circle.TStart > circle.TEnd, span.Reversed)
	}
	for _, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, admitted, sourceErr := loftdirect.CertifyLoftSourcePartition(t.Context(), bodyNode(pair[0]), bodyNode(pair[1]),
			proofbound.NewWorkBudget(t.Context()), false)
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
		_, admitted, sourceErr = loftdirect.CertifyLoftSourcePartition(t.Context(), bodyNode(pair[0]), bodyNode(pair[1]), stopAtReader, false)
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
			_, accepted := loftdirect.LoftSourceCapSpanOf(nil, provenance.RoleCapStart, 0, circle)
			require.False(t, accepted)
		})
	}
}

func TestLoftFitSplineSourceCapsRealWedge(t *testing.T) {
	world, base, top := wedgePlanes(t)
	lowerSketch, lowerProfile := wedgeSplineSketch(t, world, base)
	upperSketch, upperProfile := wedgeSplineSketch(t, world, top)
	loft, err := Loft(t.Context(), New(), lowerSketch, lowerProfile, upperSketch, upperProfile)
	require.NoError(t, err)

	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	require.False(t, prism.n.Doc == loft.n.Doc)
	require.Equal(t, prism.n.Origin.Producer, loft.n.Origin.Producer)

	budget := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
	spans, accepted, err := loftdirect.CertifyLoftSourceSpanCaps(t.Context(), budget, bodyNode(prism), bodyNode(loft))
	require.NoError(t, err)
	require.True(t, accepted)
	require.True(t, spans.Prism == bodyNode(prism) && spans.Loft == bodyNode(loft))
	require.Positive(t, spans.Visits)
	require.Less(t, spans.Visits, uint64(4096))
	lp := loft.n.Payload.(loftPayload)
	for cap := range 2 {
		want := lp.Profile0.Outer.Segments
		role := provenance.RoleCapStart
		if cap == 1 {
			want = lp.Profile1.Outer.Segments
			role = provenance.RoleCapEnd
		}
		require.Len(t, spans.LoftCaps[cap], 3)
		var fitCount int
		for i, span := range spans.LoftCaps[cap] {
			require.True(t, span.Body == bodyNode(loft))
			require.Equal(t, i, span.Segment)
			require.Equal(t, loft.n.Origin.Producer, span.CapFace.Producer)
			require.Equal(t, role, span.CapFace.Role)
			require.NotEmpty(t, span.WallFaces)
			for _, face := range span.WallFaces {
				require.Equal(t, loft.n.Origin.Producer, face.Producer)
			}
			fit, isFit := span.Curve.(sectionrecord.FitSplineSeg)
			if !isFit {
				continue
			}
			fitCount++
			require.Len(t, fit.Fit, 5)
			require.Equal(t, want[i], span.Curve)
			require.Equal(t, float64(0), span.TStart)
			require.Equal(t, float64(1), span.TEnd)
			require.False(t, span.Reversed)
		}
		require.Equal(t, 1, fitCount)
	}
	for _, order := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, admitted, sourceErr := loftdirect.CertifyLoftSourcePartition(t.Context(), bodyNode(order[0]), bodyNode(order[1]),
			proofbound.NewWorkBudget(t.Context()), false)
		require.NoError(t, sourceErr)
		require.False(t, admitted)
		boundary, outputAdmitted, outputErr := loftdirect.CertifyLoftOutputBoundaryForDirectShell(t.Context(),
			bodyNode(order[0]), bodyNode(order[1]), proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, outputErr)
		require.False(t, outputAdmitted)
		require.Empty(t, boundary.Shell.Triangles)
	}

	shortBudget := proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context()))
	partial, accepted, err := loftdirect.CertifyLoftSourceSpanCaps(t.Context(), shortBudget, bodyNode(prism), bodyNode(loft))
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.False(t, accepted)
	require.True(t, partial.Prism == nil && partial.Loft == nil)

	canceled, cancel := context.WithCancel(t.Context())
	steps := 0
	operation := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 16 {
				cancel()
			}
			return canceled.Err()
		},
		ErrFn: canceled.Err,
	}
	partial, accepted, err = loftdirect.CertifyLoftSourceSpanCaps(canceled,
		proofbound.NewWallWorkBudgetWithOperation(4096, operation), bodyNode(prism), bodyNode(loft))
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, accepted)
	require.True(t, partial.Prism == nil && partial.Loft == nil)
	require.Equal(t, 16, steps)
}
