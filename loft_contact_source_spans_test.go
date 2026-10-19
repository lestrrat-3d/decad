package decad

import (
	"context"
	"math"
	"testing"

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
	certificate, certified, certErr := certifyLoftSourceSpanCaps(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		prism, loft)
	require.NoError(t, certErr)
	require.True(t, certified)
	require.True(t, certificate.prism == prism && certificate.loft == loft)
	require.Equal(t, [3]string{"0", "10", "12"}, [3]string{
		certificate.levels[0].RatString(), certificate.levels[1].RatString(),
		certificate.levels[2].RatString(),
	})
	require.Positive(t, certificate.visits)
	require.Less(t, certificate.visits, uint64(4096))
	for cap := range 2 {
		require.Equal(t, 4, len(certificate.prismCaps[cap]))
		require.Equal(t, 3, len(certificate.loftCaps[cap]))
		var arcsWithWalls int
		for _, span := range certificate.loftCaps[cap] {
			require.True(t, span.body == loft)
			require.Equal(t, float64(0), span.tStart)
			require.Equal(t, float64(1), span.tEnd)
			if _, arc := span.curve.(sectionrecord.ArcSeg); arc {
				require.Positive(t, len(span.wallFaces))
				arcsWithWalls++
			}
		}
		require.Equal(t, 1, arcsWithWalls)
	}
	ordered, certifiedOrder, orderErr := certifyLoftCapSourceOrder(t.Context(),
		proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
		certificate)
	require.NoError(t, orderErr)
	require.True(t, certifiedOrder)
	require.Positive(t, ordered.visits)
	for cap := range 2 {
		require.NotEmpty(t, ordered.caps[cap])
		var sharedCells, cutArcs int
		for _, cell := range ordered.caps[cap] {
			require.NotEmpty(t, cell.edges)
			if cell.prism && cell.loft {
				sharedCells++
			}
			for _, edge := range cell.edges {
				require.True(t, edge.span.body == prism || edge.span.body == loft)
				require.Less(t, edge.tStart, edge.tEnd)
				require.Equal(t, edge.span.body.n.Origin.Producer,
					edge.span.capFace.Producer)
				if _, arc := edge.span.curve.(sectionrecord.ArcSeg); arc && edge.partial {
					require.True(t, edge.span.body == loft)
					cutArcs++
				}
			}
		}
		require.Positive(t, sharedCells)
		require.Positive(t, cutArcs)
	}
	_, certifiedOrder, orderErr = certifyLoftCapSourceOrder(t.Context(),
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
	_, certifiedOrder, orderErr = certifyLoftCapSourceOrder(canceled,
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
		arcSource, sourceOK, sourceErr := certifyLoftSourceSpanCaps(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context())),
			prism, arcLoft)
		require.NoError(t, sourceErr)
		require.True(t, sourceOK)
		require.Len(t, arcSource.loftCaps[0], arcs+2)
		arcCount := 0
		for _, span := range arcSource.loftCaps[0] {
			if _, isArc := span.curve.(sectionrecord.ArcSeg); isArc {
				arcCount++
			}
		}
		require.Equal(t, arcs, arcCount)
		conservativeSegments := len(arcSource.prismCaps[0]) +
			len(arcSource.loftCaps[0]) - arcCount + 256*arcCount
		require.Greater(t, conservativeSegments, prismcells.MaxArrangementSegments)
		steps := 0
		operation := &proofbound.WorkBudget{
			StepFn: func() error { steps++; return nil },
			ErrFn:  t.Context().Err,
		}
		_, admitted, capErr := certifyLoftCapSourceOrder(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(4096, operation), arcSource)
		require.NoError(t, capErr)
		require.False(t, admitted)
		require.Positive(t, steps)
		require.Less(t, steps,
			len(arcSource.prismCaps[0])+len(arcSource.loftCaps[0]))
	})
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			source, admitted, sourceErr := certifyLoftSourcePartition(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, sourceErr)
			require.False(t, admitted)
			require.Zero(t, source.SourceVisits)

			boundary, accepted, boundaryErr := certifyLoftOutputBoundary(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
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
	spans, accepted, err := certifyLoftSourceSpanCaps(t.Context(), budget, prism, loft)
	require.NoError(t, err)
	require.True(t, accepted)
	require.True(t, spans.prism == prism && spans.loft == loft)
	require.Positive(t, spans.visits)
	require.Less(t, spans.visits, uint64(4096))
	lp := loft.n.Payload.(loftPayload)
	for cap := range 2 {
		want := lp.Profile0.Outer.Segments
		role := provenance.RoleCapStart
		if cap == 1 {
			want = lp.Profile1.Outer.Segments
			role = provenance.RoleCapEnd
		}
		require.Len(t, spans.loftCaps[cap], 3)
		var fitCount int
		for i, span := range spans.loftCaps[cap] {
			require.True(t, span.body == loft)
			require.Equal(t, i, span.segment)
			require.Equal(t, loft.n.Origin.Producer, span.capFace.Producer)
			require.Equal(t, role, span.capFace.Role)
			require.NotEmpty(t, span.wallFaces)
			for _, face := range span.wallFaces {
				require.Equal(t, loft.n.Origin.Producer, face.Producer)
			}
			fit, isFit := span.curve.(sectionrecord.FitSplineSeg)
			if !isFit {
				continue
			}
			fitCount++
			require.Len(t, fit.Fit, 5)
			require.Equal(t, want[i], span.curve)
			require.Equal(t, float64(0), span.tStart)
			require.Equal(t, float64(1), span.tEnd)
			require.False(t, span.reversed)
		}
		require.Equal(t, 1, fitCount)
	}
	for _, order := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, admitted, sourceErr := certifyLoftSourcePartition(t.Context(), order[0], order[1],
			proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, sourceErr)
		require.False(t, admitted)
		boundary, outputAdmitted, outputErr := certifyLoftOutputBoundaryForDirectShell(t.Context(),
			order[0], order[1], proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, outputErr)
		require.False(t, outputAdmitted)
		require.Empty(t, boundary.Shell.Triangles)
	}

	shortBudget := proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context()))
	partial, accepted, err := certifyLoftSourceSpanCaps(t.Context(), shortBudget, prism, loft)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.False(t, accepted)
	require.True(t, partial.prism == nil && partial.loft == nil)

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
	partial, accepted, err = certifyLoftSourceSpanCaps(canceled,
		proofbound.NewWallWorkBudgetWithOperation(4096, operation), prism, loft)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, accepted)
	require.True(t, partial.prism == nil && partial.loft == nil)
	require.Equal(t, 16, steps)
}

func TestLoftFitSplineSourcePreflightRefusesOversizedInput(t *testing.T) {
	fit := sectionrecord.FitSplineSeg{Fit: make([]sectionrecord.Point2, 65), TStart: 0, TEnd: 1}
	budget := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
	visits, accepted, err := loftSourceFitPreflight(budget, nil,
		[]sectionrecord.CurveSegment{fit}, nil)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.False(t, accepted)
	require.Zero(t, visits)
}
