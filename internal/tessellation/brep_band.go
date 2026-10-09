package tessellation

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// BandWallInput names the shared side-ring samples and the receiving wall.
type BandWallInput struct {
	FaceEmbed, WallEmbed  brepgeom.Embed
	WallSegment           sectionrecord.CurveSegment
	WallWalk              survey2d.SegmentWalk
	WalkIndex             int
	SideLevel, WallHeight float64
	Counts, SideStarts    []int
	SidePoints            []sectionrecord.Point2
	SideBounds            []proofbound.WalkEndBound
	SideSag               []float64
}

// ImposeBandWall orders a band's side-ring points along the receiving wall.
// A mismatched end or direction refuses instead of yielding unshared vertices.
func ImposeBandWall[F comparable](in BandWallInput) (ChordSamples[F], error) {
	refuse := func(why string) error {
		return fmt.Errorf(`%w: a chamfer band's side contour and the wall beside it %s`, decaderr.ErrUnsupported, why)
	}
	i, w := in.WalkIndex, in.WallWalk
	count := in.Counts[i]
	ring := func(k int) (sectionrecord.Point2, proofbound.WalkEndBound) {
		idx := in.SideStarts[i] + k
		if k == count {
			idx = in.SideStarts[(i+1)%len(in.Counts)]
		}
		c := in.FaceEmbed.Canon(in.SidePoints[idx].U, in.SidePoints[idx].V, in.SideLevel)
		l := in.WallEmbed.Local(c)
		return sectionrecord.Point2{U: l[0], V: l[1]}, in.SideBounds[idx]
	}
	order := make([]int, 0, count)
	if !w.Closed {
		start, end := sectionrecord.Point2{U: w.StartU, V: w.StartV}, sectionrecord.Point2{U: w.EndU, V: w.EndV}
		p0, _ := ring(0)
		pn, _ := ring(count)
		switch {
		case p0 == start && pn == end:
			for k := range count {
				order = append(order, k)
			}
		case pn == start && p0 == end:
			for k := count; k >= 1; k-- {
				order = append(order, k)
			}
		default:
			return ChordSamples[F]{}, refuse(`disagree on the wall's ends`)
		}
	} else {
		p0, _ := ring(0)
		p1, _ := ring(1)
		cross := (p0.U-w.CU)*(p1.V-w.CV) - (p0.V-w.CV)*(p1.U-w.CU)
		if cross == 0 || !w.IsCircular() {
			return ChordSamples[F]{}, refuse(`disagree on the wall's direction`)
		}
		order = append(order, 0)
		if (cross > 0) == (w.Th1 > w.Th0) {
			for k := 1; k < count; k++ {
				order = append(order, k)
			}
		} else {
			for k := count - 1; k >= 1; k-- {
				order = append(order, k)
			}
		}
	}
	out := ChordSamples[F]{Walks: 1}
	for j, k := range order {
		p, bound := ring(k)
		if j == 0 && !w.Closed {
			bound = boundarywalk.DenotedStartBound(in.WallSegment, w)
		}
		out.Samples = append(out.Samples, p)
		out.BoundOf = append(out.BoundOf, bound)
	}
	if w.IsCircular() {
		out.MaxSag = in.SideSag[i]
		out.WallSlack = proofbound.ProductUpper(WalkWallSlack(w, count, in.WallHeight), 1+1e-9)
		out.CapSlack = proofbound.ProductUpper(WalkSegmentArea(w, count), 1+1e-9)
		out.SegmentArea = WalkSegmentArea(w, count)
	}
	return out, nil
}

// CurvedMiterInput names the offset carriers and held radius of a fillet band.
type CurvedMiterInput struct {
	Walks                   []survey2d.SideWalk
	Curved                  []bool
	CapWallStart            []int
	Radius                  proofbound.RatInterval
	HeldRadius              float64
	CapLevel, CapLevelDelta float64
	MaterialSign            float64
}

// CurvedMiterRings replaces affine corner stations with offset-carrier feet.
// It returns each seam's chord gap for the patch displacement proof.
func CurvedMiterRings(points [][]FilletRingPoint, in CurvedMiterInput) ([]float64, error) {
	n := len(points) - 1
	rUpper := proofbound.RatFloatUp(in.Radius.Hi)
	level, levelDelta := proofarith.FloatRat(in.CapLevel), proofarith.FloatRat(in.CapLevelDelta)
	if level == nil || levelDelta == nil || proofbound.IsNonFinite(rUpper) {
		return nil, fmt.Errorf(`%w: a curved fillet's radius or cap level is not finite`, decaderr.ErrNotFinite)
	}
	levelIv := proofbound.IntervalWiden(proofbound.PointInterval(level), levelDelta)
	one := proofbound.PointInterval(big.NewRat(1, 1))
	m := big.NewRat(int64(in.MaterialSign), 1)
	seamGap := make([]float64, len(in.Curved))
	for corner, curved := range in.Curved {
		if !curved {
			continue
		}
		prev := in.Walks[(corner+len(in.Walks)-1)%len(in.Walks)]
		cur := in.Walks[corner]
		speed, ok := capcontour.MiterLocusSpeedUpper(prev, cur, 0, rUpper, cur.StartU, cur.StartV)
		if !ok {
			return nil, fmt.Errorf(`%w: a curved fillet's offset locus folds`, decaderr.ErrUnsupported)
		}
		gap, ok := filletband.CurvedMiterChordGap(prev, cur, rUpper, n)
		if !ok {
			return nil, fmt.Errorf(`%w: a curved fillet's seam chord has no bound`, decaderr.ErrUnsupported)
		}
		seamGap[corner] = proofbound.RatFloatUp(gap)
		ci := in.CapWallStart[corner]
		for k := 1; k < n; k++ {
			phiIv := proofbound.IntervalScale(proofbound.HalfPiInterval(), big.NewRat(int64(k), int64(n)))
			sinIv, cosIv, ok := proofbound.RadSinCosSpan(phiIv)
			if !ok {
				return nil, fmt.Errorf(`%w: a curved fillet's ring angle has no enclosure`, decaderr.ErrUnsupported)
			}
			tIv := proofbound.IntervalMul(in.Radius, proofbound.IntervalSub(one, cosIv))
			phi := float64(k) * (math.Pi / 2) / float64(n)
			tHeld := in.HeldRadius * (1 - math.Cos(phi))
			p, ok := filletband.CurvedMiterPoint(prev, cur, in.HeldRadius, tHeld, cur.StartU, cur.StartV)
			if !ok {
				return nil, fmt.Errorf(`%w: a curved fillet's ring has no intersection foot`, decaderr.ErrUnsupported)
			}
			shift := proofbound.ProductUpper(speed, proofbound.IntervalFloatError(tIv, tHeld))
			u, uBound := heldMiterCoordinate(p[0])
			v, vBound := heldMiterCoordinate(p[1])
			zIv := proofbound.IntervalAdd(levelIv, proofbound.IntervalScale(
				proofbound.IntervalMul(in.Radius, proofbound.IntervalSub(one, sinIv)), m))
			z := points[k][ci].Z
			zBound := proofbound.IntervalFloatError(zIv, z)
			delta := proofbound.AbsSumUpper(proofbound.Radius2D(
				proofbound.AbsSumUpper(uBound, shift), proofbound.AbsSumUpper(vBound, shift)), zBound)
			points[k][ci] = FilletRingPoint{Point: sectionrecord.Point2{U: u, V: v}, Z: z, Delta: delta}
		}
	}
	return seamGap, nil
}

func heldMiterCoordinate(iv proofbound.RatInterval) (float64, float64) {
	held := brepgeom.Held(iv)
	return held, proofbound.IntervalFloatError(iv, held)
}
