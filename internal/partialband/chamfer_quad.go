package partialband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ChamferQuadExact restricts the admission of one partial-chamfer quad to
// exactly coplanar recorded floats whose cap line is dcFloat times the travel
// length off the side line and whose side level is dsFloat below it. A nonzero
// contour/level displacement is refused by the caller.
func ChamferQuadExact(points [4][3]float64, dcFloat, dsFloat float64, e brepgeom.Embed) error {
	r := chamferRats(points)
	a, b, c, d := r[0], r[1], r[2], r[3]
	ab, ac, ad := chamferSub(b, a), chamferSub(c, a), chamferSub(d, a)
	n := chamferCross(ab, ac)
	if chamferDot(n, ad).Sign() != 0 || chamferDot(n, n).Sign() == 0 {
		return fmt.Errorf(`%w: a partial chamfer patch is not one exact plane`, decaderr.ErrUnsupported)
	}
	// A concave or self-crossing quadrilateral would make the two-triangle
	// area and divergence fan differ from the patch's actual boundary.
	second := chamferCross(chamferSub(c, a), ad)
	if chamferDot(n, second).Sign() <= 0 {
		return fmt.Errorf(`%w: a partial chamfer patch has inconsistent triangle winding`, decaderr.ErrUnsupported)
	}
	// The cap line must be the exact offset of the side carrier. Its
	// endpoints may have different travel coordinates after a miter, so
	// compare the two line equations rather than endpoint differences.
	local := [4][3]*big.Rat{}
	for i, p := range points {
		v := e.Local(p)
		for k := range 3 {
			local[i][k] = proofarith.FloatRat(v[k])
		}
	}
	travel := chamferSub(local[1], local[0])
	if travel[2].Sign() != 0 || (travel[0].Sign() != 0) == (travel[1].Sign() != 0) {
		return fmt.Errorf(`%w: a partial chamfer's selected line is not along a face axis`, decaderr.ErrUnsupported)
	}
	length := new(big.Rat).Abs(travel[0])
	if length.Sign() == 0 {
		length.Abs(travel[1])
	}
	cross2 := func(v [3]*big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(travel[0], v[1]),
			new(big.Rat).Mul(travel[1], v[0]))
	}
	dc := proofarith.FloatRat(dcFloat)
	want := new(big.Rat).Mul(dc, length)
	for _, capPoint := range [][3]*big.Rat{local[2], local[3]} {
		if cross2(chamferSub(capPoint, local[0])).Cmp(want) != 0 {
			return fmt.Errorf(`%w: a partial chamfer's cap line is not the exact requested offset`, decaderr.ErrUnsupported)
		}
	}
	level := new(big.Rat).Abs(new(big.Rat).Sub(local[0][2], local[2][2]))
	if level.Cmp(proofarith.FloatRat(dsFloat)) != 0 {
		return fmt.Errorf(`%w: a partial chamfer's side level is not the exact requested setback`, decaderr.ErrUnsupported)
	}
	return nil
}

func chamferRats(points [4][3]float64) [4][3]*big.Rat {
	var out [4][3]*big.Rat
	for i := range 4 {
		for k := range 3 {
			out[i][k] = proofarith.FloatRat(points[i][k])
		}
	}
	return out
}

func chamferSub(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for k := range 3 {
		out[k] = new(big.Rat).Sub(a[k], b[k])
	}
	return out
}

func chamferCross(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

func chamferDot(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for k := range 3 {
		out.Add(out, new(big.Rat).Mul(a[k], b[k]))
	}
	return out
}

// ChamferQuadMass returns one quad's area, three-times-volume
// flux and first-moment flux in the loop's geometric winding.
func ChamferQuadMass(points [4][3]float64) (proofbound.RatInterval, *big.Rat, [3]*big.Rat, error) {
	r := chamferRats(points)
	vol3 := new(big.Rat)
	var moments [3]*big.Rat
	for k := range 3 {
		moments[k] = new(big.Rat)
	}
	for _, pair := range [][2]int{{1, 2}, {2, 3}} {
		a, b, c := r[0], r[pair[0]], r[pair[1]]
		cross := chamferCross(chamferSub(b, a), chamferSub(c, a))
		centroid3 := [3]*big.Rat{}
		for k := range 3 {
			centroid3[k] = new(big.Rat).Add(a[k], b[k])
			centroid3[k].Add(centroid3[k], c[k])
		}
		vol3.Add(vol3, new(big.Rat).Quo(chamferDot(centroid3, cross), big.NewRat(6, 1)))
		for k := range 3 {
			sum := new(big.Rat)
			for _, p := range [][2]int{{0, 0}, {1, 1}, {2, 2}, {0, 1}, {1, 2}, {2, 0}} {
				v := [3]*big.Rat{a[k], b[k], c[k]}
				sum.Add(sum, new(big.Rat).Mul(v[p[0]], v[p[1]]))
			}
			moments[k].Add(moments[k], new(big.Rat).Quo(new(big.Rat).Mul(cross[k], sum), big.NewRat(24, 1)))
		}
	}
	// The two fan triangles lie in the same plane and share orientation.
	// Their area vectors are parallel, so the total area is half the sum of
	// their exact cross-vector lengths. For a trapezoid those vectors are
	// proportional, and each root is enclosed independently.
	area := proofbound.PointInterval(new(big.Rat))
	for _, pair := range [][2]int{{1, 2}, {2, 3}} {
		cross := chamferCross(chamferSub(r[pair[0]], r[0]), chamferSub(r[pair[1]], r[0]))
		root, ok := proofbound.SqrtFixed(chamferDot(cross, cross))
		if !ok {
			return proofbound.RatInterval{}, nil, [3]*big.Rat{}, fmt.Errorf(`%w: a chamfer patch has no area`, decaderr.ErrUnsupported)
		}
		area = proofbound.IntervalAdd(area, proofbound.IntervalScale(root, big.NewRat(1, 2)))
	}
	return area, vol3, moments, nil
}
