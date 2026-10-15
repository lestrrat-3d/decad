package decad

import (
	"fmt"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesVerticalRefusesChangedBlankSection(t *testing.T) {
	blank := sectionSketchBody(t, func(s *sketch.Sketch) {
		section := []planePoint{{U: 2}, {U: 5}, {U: 5, V: 3}, {U: 2, V: 1}}
		points := make([]*sketch.Point, len(section))
		for i, point := range section {
			points[i] = s.CreatePoint(point.U, point.V)
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}, FullRevolution{})
	candidates := revolveRootCandidates(blank)
	require.Len(t, candidates, 1)
	proof := &revolveLoftFacesSourceCertificate{blank: blank, section: candidates[0]}
	require.True(t, revolveLoftFacesSectionCurrent(proof))
	index := -1
	for i, span := range proof.section.Spans {
		if span.Start.U == 5 && span.End.U == 5 {
			index = i
			break
		}
	}
	require.NotEqual(t, -1, index)
	proof.section.Spans[index].Start.U = 2
	proof.section.Spans[index].End.U = 2
	proof.section.Spans[index].End.V = 2
	require.False(t, revolveLoftFacesSectionCurrent(proof),
		"the retained face cannot authenticate changed section endpoints")
}

func TestRevolveLoftFacesConstantXRejectsSubstitutedSourceRef(t *testing.T) {
	const segmentCount = 2
	trim := &loftFacesTrimRecord{
		Source: &pointSectionRecord{profile: profileRecord{Outer: loopRecord{
			Segments: []curveSegment{lineSeg{}, lineSeg{}},
		}}},
		Maps: make([]loftFaceMap, 3),
	}
	trim.Base.Groups = make([]facetGroup, 2*segmentCount+2)
	bound := make([][]provenance.Ref, 2)
	for band := range bound {
		bound[band] = make([]provenance.Ref, segmentCount)
		for segment := range bound[band] {
			ref := provenance.Ref{Producer: 7, Role: fmt.Sprintf("side(%d,%d)", band, segment)}
			trim.Base.Groups[band*segmentCount+segment].Origins = []provenance.Ref{ref}
			bound[band][segment] = ref
		}
	}
	tooth := bornBody(&Body{})
	require.True(t, revolveLoftFacesSourceRefsCurrent(trim, tooth, bound))
	bound[0][0] = bound[0][1]
	require.False(t, revolveLoftFacesSourceRefsCurrent(trim, tooth, bound),
		"a retained cell cannot name the neighboring source segment")
}

func TestRevolveLoftFacesVerticalConstantXAndTrimLimits(t *testing.T) {
	zero, one := lfZero(), lfOne()
	mapped := loftFaceMap{
		Constant: loftFaceRatVec{big.NewRat(5, 1), zero, zero},
		Q0:       loftFaceRatVec{big.NewRat(2, 1), zero, zero},
		QU:       loftFaceRatVec{big.NewRat(6, 1), one, zero},
		QV:       loftFaceRatVec{big.NewRat(4, 1), zero, one},
		G0:       one, GU: big.NewRat(3, 1), GV: big.NewRat(2, 1),
	}
	x, ok := loftface.ConstantX(mapped)
	require.True(t, ok)
	require.Equal(t, "7", x.RatString())
	mapped.QU[0] = big.NewRat(5, 1)
	_, ok = loftface.ConstantX(mapped)
	require.False(t, ok)

	trim := &loftFacesTrimRecord{
		Toe:  &pointConeLimit{apex: big.NewRat(13, 2), far: big.NewRat(-2, 1), slope: one},
		Heel: &pointConeLimit{apex: big.NewRat(8, 1), far: big.NewRat(-2, 1), slope: one},
	}
	section := revolveSectionSpan{Start: planePoint{U: 6}, End: planePoint{U: 6, V: 3.5}}
	lower, upper, outcome := revolveLoftFacesVerticalLimits(trim, section, big.NewRat(6, 1))
	require.Equal(t, revolveLoftFacesVerticalContact, outcome)
	require.Equal(t, "1/2", lower.RatString())
	require.Equal(t, "2", upper.RatString())
	section.Start.U, section.End.U = 3.2, 3.2
	lower, upper, outcome = revolveLoftFacesVerticalLimits(trim, section, big.NewRat(16, 5))
	require.Equal(t, revolveLoftFacesVerticalContact, outcome)
	require.Equal(t, 0, lower.Cmp(big.NewRat(33, 10)))
	require.Equal(t, "7/2", upper.RatString())
	section.End.V = 3.2
	lower, upper, outcome = revolveLoftFacesVerticalLimits(trim, section, lfF(3.2))
	require.Equal(t, revolveLoftFacesVerticalMiss, outcome)
	require.Greater(t, lower.Cmp(upper), 0, "the toe tool leaves a strict whole-side gap")
	section.Start.U, section.End.U, section.End.V = 3.25, 3.25, 3.25
	_, _, outcome = revolveLoftFacesVerticalLimits(trim, section, big.NewRat(13, 4))
	require.Equal(t, revolveLoftFacesVerticalRefusal, outcome,
		"a touching radial boundary does not prove a miss")
	section.Start.U, section.End.U = 8, 8
	_, _, outcome = revolveLoftFacesVerticalLimits(trim, section, big.NewRat(8, 1))
	require.Equal(t, revolveLoftFacesVerticalRefusal, outcome)
}

func TestRevolveLoftFacesConstantXBand(t *testing.T) {
	zero, one := lfZero(), lfOne()
	makeMap := func(x *big.Rat) loftFaceMap {
		return loftFaceMap{
			Constant: loftFaceRatVec{x, zero, zero},
			Q0:       loftFaceRatVec{zero, zero, zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       loftFaceRatVec{zero, zero, one},
			G0:       one, GU: zero, GV: zero,
		}
	}
	box := [4]*big.Rat{zero, one, zero, one}
	maps := []loftFaceMap{makeMap(big.NewRat(5, 1)), makeMap(big.NewRat(6, 1)), makeMap(big.NewRat(7, 1))}
	for _, tc := range []struct {
		x    *big.Rat
		band int
	}{
		{big.NewRat(11, 2), 0},
		{big.NewRat(13, 2), 1},
	} {
		band, time, ok := loftface.ConstantXBand(maps, box, tc.x)
		require.True(t, ok)
		require.Equal(t, tc.band, band)
		require.Equal(t, "1/2", time.RatString())
	}
	_, _, ok := loftface.ConstantXBand(maps, box, big.NewRat(6, 1))
	require.False(t, ok, "a cap coincidence has no unique interior band time")
	maps[2] = makeMap(big.NewRat(11, 2))
	_, _, ok = loftface.ConstantXBand(maps, box, big.NewRat(21, 4))
	require.False(t, ok, "cap X must strictly increase")
	maps[2] = makeMap(big.NewRat(7, 1))
	maps[1].QU[0] = one
	_, _, ok = loftface.ConstantXBand(maps, box, big.NewRat(11, 2))
	require.False(t, ok, "cap X must be source independent")
}

func TestRevolveLoftFacesVerticalClipsLiveSourceSpan(t *testing.T) {
	zero, one := lfZero(), lfOne()
	makeMap := func(x *big.Rat) loftFaceMap {
		return loftFaceMap{
			Constant: loftFaceRatVec{x, zero, zero},
			Q0:       loftFaceRatVec{zero, zero, zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       loftFaceRatVec{zero, zero, one},
			G0:       one, GU: zero, GV: zero,
		}
	}
	span := freeform.BezierSpan{{U: one, V: zero}, {U: big.NewRat(3, 1), V: zero}}
	trim := &loftFacesTrimRecord{Maps: []loftFaceMap{makeMap(big.NewRat(5, 1)), makeMap(big.NewRat(6, 1))}}
	cell, ok := revolveLoftFacesVerticalBezier(trim, 3, 0, 0,
		big.NewRat(1, 2), big.NewRat(3, 2), big.NewRat(5, 2), provenance.Ref{}, span)
	require.True(t, ok)
	require.Zero(t, cell.lower.Side)
	require.Equal(t, -1, cell.lower.Before)
	require.Equal(t, 1, cell.lower.After)
	require.Zero(t, cell.upper.Side)
	require.Equal(t, -1, cell.upper.Before)
	require.Equal(t, 1, cell.upper.After)
	require.Less(t, cell.lower.Root.Hi.Cmp(cell.upper.Root.Lo), 0)
	require.Len(t, cell.retained, 1)
	require.Equal(t, "1/4", cell.lower.Root.Hi.RatString())
	rootPoint := loftface.BezierPoint(span, cell.retained[0].Source.Lo)
	require.Equal(t, "3/2", rootPoint[0].RatString())
	require.Equal(t, 0, cell.retained[0].Source.Lo.Cmp(cell.lower.Root.Hi))
	require.Equal(t, 0, cell.retained[0].Source.Hi.Cmp(cell.upper.Root.Lo))
	require.True(t, cell.retained[0].LoOpen,
		"an exact radial clip root is not inside the strict retained interval")
	require.False(t, cell.retained[0].HiOpen)
	require.Equal(t, "11/2", cell.retained[0].Box[0].Lo.RatString())
	require.Equal(t, "11/2", cell.retained[0].Box[0].Hi.RatString())
	for _, limit := range [...]loftface.VerticalLimit{cell.lower, cell.upper} {
		require.Less(t, limit.Root.Lo.Cmp(limit.Root.Hi), 0)
		require.Equal(t, "11/2", limit.Box[0].Lo.RatString())
	}
	_, ok = loftface.VerticalClassify([]*big.Rat{zero, one})
	require.False(t, ok)
}

func TestRevolveLoftFacesVerticalAffineOrientation(t *testing.T) {
	zero, one := lfZero(), lfOne()
	makeMap := func(scale *big.Rat) loftFaceMap {
		return loftFaceMap{
			Constant: loftFaceRatVec{zero, zero, zero},
			Q0:       loftFaceRatVec{zero, zero, zero},
			QU:       loftFaceRatVec{zero, scale, zero},
			QV:       loftFaceRatVec{zero, zero, scale},
			G0:       one, GU: zero, GV: zero,
		}
	}
	first, last := makeMap(big.NewRat(1, 2)), makeMap(one)
	affine, ok := loftface.VerticalAffineMap(first, last, big.NewRat(1, 2))
	require.True(t, ok)
	require.Equal(t, "3/4", affine.Column[0][0].RatString())
	require.Equal(t, "3/4", affine.Column[1][1].RatString())
	require.Equal(t, "9/16", affine.Det.RatString())
	last.QV[2] = big.NewRat(-1, 1)
	_, ok = loftface.VerticalAffineMap(first, last, one)
	require.False(t, ok)
	last.QV[2] = one
	last.GU = one
	_, ok = loftface.VerticalAffineMap(first, last, big.NewRat(1, 2))
	require.False(t, ok)
}
