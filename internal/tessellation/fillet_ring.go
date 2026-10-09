package tessellation

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// FilletRingCount is the fewest strips whose quarter-circle chord sagitta
// stays within half the chord budget, with a minimum of two.
func FilletRingCount(radius, chord float64) int {
	tol := chord / 2
	if !(radius > 0) || !(tol > 0) {
		return 2
	}
	arg := 1 - tol/radius
	if arg < -1 {
		arg = -1
	}
	n := int(math.Ceil((math.Pi / 2) / (2 * math.Acos(arg))))
	return min(max(n, 2), 4096)
}

// FilletLoopInput names the shared walk and connector sample layout.
type FilletLoopInput struct {
	Walks                      []survey2d.SideWalk
	Joins                      []CapBlendJoin
	Count, ArcCount, SideStart []int
	CapSamples                 int
}

// FilletCell is one strip cell between consecutive cap samples of a patch.
type FilletCell struct{ C0, C1, Patch int }

// FilletLoopLayout pairs cap samples with side samples and lists patch cells.
func FilletLoopLayout(in FilletLoopInput) ([]int, []FilletCell, int, error) {
	n := len(in.Walks)
	N := in.CapSamples
	var match []int
	var cells []FilletCell
	c, p := 0, 0
	for i, w := range in.Walks {
		nw := 1
		if w.IsCircular() {
			nw = in.Count[i]
		}
		for k := range nw {
			match = append(match, in.SideStart[i]+k)
			cells = append(cells, FilletCell{c, c + 1, p})
			c++
		}
		p++
		ni := (i + 1) % n
		if in.Joins != nil && in.Joins[ni].Arc {
			for range in.ArcCount[ni] {
				match = append(match, in.SideStart[ni])
				cells = append(cells, FilletCell{c, c + 1, p})
				c++
			}
			p++
		}
	}
	if len(match) != N {
		return nil, nil, 0, fmt.Errorf(`%w: fillet band's cap ring holds %d samples for %d matched side samples`,
			decaderr.ErrUnsupported, N, len(match))
	}
	if c != N {
		return nil, nil, 0, fmt.Errorf(`%w: fillet band cells cover %d of %d cap samples`, decaderr.ErrUnsupported, c, N)
	}
	if len(cells) > 0 {
		cells[len(cells)-1].C1 = 0
	}
	return match, cells, p, nil
}

// FilletRingInput names corresponding side and cap samples and their level
// and radius enclosures. MaterialSign is the sign of the cap's material side.
type FilletRingInput struct {
	Side, Cap               []sectionrecord.Point2
	Radius, RadiusDelta     float64
	CapLevel, CapLevelDelta float64
	SideLevel               float64
	MaterialSign            float64
	Count                   int
	Noun, RingNoun          string
}

// FilletRingPoint is one interior ring sample and its proven displacement.
type FilletRingPoint struct {
	Point    sectionrecord.Point2
	Z, Delta float64
}

// FilletRingGeometry builds the interior rings of a quarter-circle fillet.
// Each point's Delta includes interpolation, radius and level rounding.
func FilletRingGeometry(in FilletRingInput) ([][]FilletRingPoint, error) {
	if len(in.Side) != len(in.Cap) {
		return nil, fmt.Errorf(`%w: %s has mismatched side and cap samples`, decaderr.ErrUnsupported, in.Noun)
	}
	rz0, rzd := proofarith.FloatRat(in.CapLevel), proofarith.FloatRat(in.CapLevelDelta)
	rr, rrd := proofarith.FloatRat(in.Radius), proofarith.FloatRat(in.RadiusDelta)
	if rz0 == nil || rzd == nil || rr == nil || rrd == nil {
		return nil, fmt.Errorf(`%w: %s's radius or level is not finite`, decaderr.ErrNotFinite, in.Noun)
	}
	rIv := proofbound.IntervalWiden(proofbound.PointInterval(rr), rrd)
	one := proofbound.PointInterval(big.NewRat(1, 1))
	mRat := big.NewRat(int64(in.MaterialSign), 1)
	rings := make([][]FilletRingPoint, in.Count+1)
	for k := 1; k < in.Count; k++ {
		phi := float64(k) * (math.Pi / 2) / float64(in.Count)
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(phi))
		if !ok {
			return nil, fmt.Errorf(`%w: %s ring's angle has no sine enclosure`, decaderr.ErrUnsupported, in.RingNoun)
		}
		fraction := 1 - math.Cos(phi)
		fRat := proofarith.FloatRat(fraction)
		errF := proofbound.IntervalFloatError(proofbound.IntervalSub(one, cosIv), fraction)
		h := in.Radius * math.Sin(phi)
		z := in.SideLevel - in.MaterialSign*h
		zIv := proofbound.IntervalWiden(proofbound.IntervalAdd(proofbound.PointInterval(rz0),
			proofbound.IntervalScale(proofbound.IntervalMul(rIv, proofbound.IntervalSub(one, sinIv)), mRat)), rzd)
		errZ := proofbound.IntervalFloatError(zIv, z)
		if fRat == nil || proofbound.IsNonFinite(errF) || proofbound.IsNonFinite(errZ) {
			return nil, fmt.Errorf(`%w: %s ring's position is not finite`, decaderr.ErrNotFinite, in.RingNoun)
		}
		rings[k] = make([]FilletRingPoint, len(in.Cap))
		for c, capPoint := range in.Cap {
			s := in.Side[c]
			u, ru, okU := filletInterpolate(s.U, capPoint.U, fraction, fRat)
			v, rv, okV := filletInterpolate(s.V, capPoint.V, fraction, fRat)
			if !okU || !okV {
				return nil, fmt.Errorf(`%w: %s ring's position is not finite`, decaderr.ErrNotFinite, in.RingNoun)
			}
			span := proofbound.AbsSumUpper(math.Abs(capPoint.U-s.U), math.Abs(capPoint.V-s.V))
			dev := proofbound.AbsSumUpper(ru, rv, proofbound.ProductUpper(errF, span), errZ)
			rings[k][c] = FilletRingPoint{Point: sectionrecord.Point2{U: u, V: v}, Z: z, Delta: dev}
		}
	}
	return rings, nil
}

// filletInterpolate returns the held interpolation and its exact rounding error.
func filletInterpolate(s, c, f float64, fRat *big.Rat) (float64, float64, bool) {
	held := s + f*(c-s)
	rs, rc := proofarith.FloatRat(s), proofarith.FloatRat(c)
	rh := proofarith.FloatRat(held)
	if rs == nil || rc == nil || rh == nil {
		return 0, 0, false
	}
	exact := new(big.Rat).Add(rs, new(big.Rat).Mul(fRat, new(big.Rat).Sub(rc, rs)))
	err := proofbound.RatFloatUp(new(big.Rat).Abs(new(big.Rat).Sub(rh, exact)))
	return held, err, true
}

// FilletMeshAreaDeficit bounds how far the held facets' total area lies from a
// surface area trueArea ± trueBound: the facets' area is enclosed in exact
// rational arithmetic over the held vertices and compared at the far ends.
// A facet the enclosure cannot state answers +Inf.
func FilletMeshAreaDeficit(verts []r3.Vec, tris [][3]int, trueArea, trueBound float64) float64 {
	sum := proofbound.PointInterval(new(big.Rat))
	for _, t := range tris {
		a, okA := proofbound.IvVec3Of(verts[t[0]])
		b, okB := proofbound.IvVec3Of(verts[t[1]])
		c, okC := proofbound.IvVec3Of(verts[t[2]])
		if !okA || !okB || !okC {
			return math.Inf(1)
		}
		cross := proofbound.IvVec3Cross(proofbound.IvVec3Sub(b, a), proofbound.IvVec3Sub(c, a))
		norm, ok := proofbound.IntervalSqrt(proofbound.IvVec3NormSq(cross))
		if !ok {
			return math.Inf(1)
		}
		sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(norm, big.NewRat(1, 2)))
	}
	ra, rb := proofarith.FloatRat(trueArea), proofarith.FloatRat(trueBound)
	if ra == nil || rb == nil {
		return math.Inf(1)
	}
	tLo, tHi := new(big.Rat).Sub(ra, rb), new(big.Rat).Add(ra, rb)
	worst := proofbound.RatMax(proofbound.RatMax(new(big.Rat).Sub(sum.Hi, tLo), new(big.Rat).Sub(tHi, sum.Lo)), new(big.Rat))
	return proofbound.RatFloatUp(worst)
}
