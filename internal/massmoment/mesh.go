package massmoment

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ErrMeshIntervalUnproved marks a mesh reading whose volume interval cannot
// prove positivity. The caller may retry at a finer mesh tolerance.
var ErrMeshIntervalUnproved = fmt.Errorf("%w: mesh mass or inertia interval is not provably positive", decaderr.ErrUnsupported)

// MeshBounds encloses the denoted solid when its mesh carries a certified
// occupied-volume difference.
type MeshBounds struct {
	Min, Max r3.Vec
	Bound    units.Value
}

// MeshIntervals contains the widened volume and centroidal moment intervals
// before density and float rounding are applied.
type MeshIntervals struct {
	Volume  proofbound.RatInterval
	Center  [3]proofbound.RatInterval
	Central [3][3]proofbound.RatInterval
	Trace   proofbound.RatInterval
}

// tetraMoments accumulates, over signed tetrahedra (O, a, b, c) with O the
// anchor, six times the volume, 24 times each first moment and 120 times each
// second moment: for one tetrahedron with det = a·(b×c) and s = a+b+c,
// 6V = det, 24P_i = det·s_i and 120Q_ij = det·(s_i s_j + Σ_v v_i v_j).
// Coordinates share one power-of-two denominator. Accumulating their integer
// numerators avoids normalizing a rational after every triangle; the sums
// round nothing.
type tetraMoments struct {
	volume6 big.Int
	first   [3]big.Int
	second  [3][3]big.Int // upper triangle, i <= j
}

func newTetraMoments() *tetraMoments {
	return new(tetraMoments)
}

// add folds one tetrahedron given its three anchored vertices and their exact
// determinant.
func (m *tetraMoments) add(a, b, c [3]*big.Int, det *big.Int) {
	m.volume6.Add(&m.volume6, det)
	var sum [3]big.Int
	var term, paired big.Int
	for i := range 3 {
		sum[i].Add(a[i], b[i])
		sum[i].Add(&sum[i], c[i])
		term.Mul(det, &sum[i])
		m.first[i].Add(&m.first[i], &term)
	}
	for i := range 3 {
		for j := i; j < 3; j++ {
			paired.Mul(&sum[i], &sum[j])
			for _, v := range [][3]*big.Int{a, b, c} {
				term.Mul(v[i], v[j])
				paired.Add(&paired, &term)
			}
			term.Mul(det, &paired)
			m.second[i][j].Add(&m.second[i][j], &term)
		}
	}
}

// moments returns V, P and the upper triangle of Q about the anchor.
func (m *tetraMoments) moments(scale *big.Int) (*big.Rat, [3]*big.Rat, [3][3]*big.Rat) {
	scale3 := new(big.Int).Exp(scale, big.NewInt(3), nil)
	scale4 := new(big.Int).Mul(scale3, scale)
	scale5 := new(big.Int).Mul(scale4, scale)
	volume := new(big.Rat).SetFrac(new(big.Int).Set(&m.volume6),
		new(big.Int).Mul(scale3, big.NewInt(6)))
	var first [3]*big.Rat
	var second [3][3]*big.Rat
	for i := range 3 {
		first[i] = new(big.Rat).SetFrac(new(big.Int).Set(&m.first[i]),
			new(big.Int).Mul(scale4, big.NewInt(24)))
		for j := i; j < 3; j++ {
			second[i][j] = new(big.Rat).SetFrac(new(big.Int).Set(&m.second[i][j]),
				new(big.Int).Mul(scale5, big.NewInt(120)))
		}
	}
	return volume, first, second
}

func tetraDet(a, b, c [3]*big.Int) *big.Int {
	var det, cross, term big.Int
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		cross.Mul(b[j], c[k])
		term.Mul(b[k], c[j])
		cross.Sub(&cross, &term)
		term.Mul(a[i], &cross)
		det.Add(&det, &term)
	}
	return &det
}

// HeldMeshIntervals integrates an audited triangle set about anchor and
// widens its moments by the occupied-volume difference and spatial extents.
func HeldMeshIntervals(ctx context.Context, bounds MeshBounds, anchor r3.Vec, verts []r3.Vec, tris [][3]int, volSymDiff float64) (MeshIntervals, error) {
	if len(verts) == 0 || len(tris) == 0 || !proofbound.FiniteVec(anchor) || !NonNegativeFinite(volSymDiff) {
		return MeshIntervals{}, fmt.Errorf("%w: mesh mass has no finite occupied-volume certificate", decaderr.ErrUnsupported)
	}
	// The binary64 vertex coordinates are exact rational inputs; only the
	// final readings round.
	anchorExact := proofarith.XptOf(anchor)
	lifted := make([]proofarith.Xpt, len(verts))
	maxScaleBits := 1
	budget := proofbound.NewWorkBudget(ctx)
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return MeshIntervals{}, err
		}
		if !proofbound.FiniteVec(v) {
			return MeshIntervals{}, fmt.Errorf("%w: mesh mass vertex is nonfinite", decaderr.ErrUnsupported)
		}
		lifted[i] = proofarith.Xsub(proofarith.XptOf(v), anchorExact)
		if bits := lifted[i].W.BitLen(); bits > maxScaleBits {
			maxScaleBits = bits
		}
	}
	// XptOf and Xsub keep dyadic coordinates over a power-of-two weight.
	// Align once, then integrate every tetrahedron over that shared scale.
	scale := new(big.Int).Lsh(big.NewInt(1), uint(maxScaleBits-1))
	vertices := make([][3]*big.Int, len(verts))
	var maxMeshNum [3]big.Int
	for i, point := range lifted {
		if err := budget.Step(); err != nil {
			return MeshIntervals{}, err
		}
		shift := uint(maxScaleBits - point.W.BitLen())
		for axis, numerator := range [3]*big.Int{point.X, point.Y, point.Z} {
			coord := new(big.Int).Lsh(numerator, shift)
			vertices[i][axis] = coord
			if abs := new(big.Int).Abs(coord); abs.Cmp(&maxMeshNum[axis]) > 0 {
				maxMeshNum[axis].Set(abs)
			}
		}
	}
	var maxMesh [3]*big.Rat
	for axis := range maxMesh {
		maxMesh[axis] = new(big.Rat).SetFrac(&maxMeshNum[axis], scale)
	}
	sums := newTetraMoments()
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return MeshIntervals{}, err
		}
		det := tetraDet(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]])
		sums.add(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]], det)
	}
	if err := budget.Err(); err != nil {
		return MeshIntervals{}, err
	}
	volume, first, second := sums.moments(scale)
	if volume.Sign() <= 0 {
		return MeshIntervals{}, fmt.Errorf("%w: mesh volume is not positive", ErrMeshIntervalUnproved)
	}
	extent, err := meshMassExtent(bounds, anchor, maxMesh)
	if err != nil {
		return MeshIntervals{}, err
	}
	// Over the symmetric difference D, |∫_D q_i| <= R_i·E and
	// |∫_D q_i q_j| <= R_i·R_j·E, since |q_i| <= R_i on both regions.
	volumeError := proofarith.FloatRat(volSymDiff)
	volumeIV := proofbound.IntervalWiden(proofbound.PointInterval(volume), volumeError)
	if volumeIV.Lo.Sign() <= 0 {
		return MeshIntervals{}, fmt.Errorf("%w: mesh volume interval includes zero", ErrMeshIntervalUnproved)
	}
	var firstIV [3]proofbound.RatInterval
	var secondIV [3][3]proofbound.RatInterval
	for i := range 3 {
		firstIV[i] = proofbound.IntervalWiden(proofbound.PointInterval(first[i]), new(big.Rat).Mul(extent[i], volumeError))
		for j := i; j < 3; j++ {
			secondError := new(big.Rat).Mul(new(big.Rat).Mul(extent[i], extent[j]), volumeError)
			secondIV[i][j] = proofbound.IntervalWiden(proofbound.PointInterval(second[i][j]), secondError)
		}
	}

	var center [3]proofbound.RatInterval
	var central [3][3]proofbound.RatInterval
	for i, origin := range []*big.Rat{proofarith.FloatRat(anchor.X), proofarith.FloatRat(anchor.Y), proofarith.FloatRat(anchor.Z)} {
		offset, _ := proofbound.IntervalQuo(firstIV[i], volumeIV)
		center[i] = proofbound.IntervalAdd(proofbound.PointInterval(origin), offset)
		for j := i; j < 3; j++ {
			shift, _ := proofbound.IntervalQuo(proofbound.IntervalMul(firstIV[i], firstIV[j]), volumeIV)
			central[i][j] = proofbound.IntervalSub(secondIV[i][j], shift)
		}
	}
	trace := proofbound.IntervalAdd(proofbound.IntervalAdd(central[0][0], central[1][1]), central[2][2])
	return MeshIntervals{Volume: volumeIV, Center: center, Central: central, Trace: trace}, nil
}

// meshMassExtent bounds |x_i - O_i| per axis for both the held mesh and the
// denoted solid. Bound widens each true coordinate from its held extreme;
// the exact rational arithmetic introduces no new rounding allowance.
func meshMassExtent(box MeshBounds, anchor r3.Vec, maxMesh [3]*big.Rat) ([3]*big.Rat, error) {
	var extent [3]*big.Rat
	allow := box.Bound.Base()
	if box.Bound.Kind() != units.Length || proofbound.IsNonFinite(allow) || allow < 0 ||
		!proofbound.FiniteVec(box.Min) || !proofbound.FiniteVec(box.Max) {
		return extent, fmt.Errorf("%w: mesh mass has no finite spatial bound", decaderr.ErrUnsupported)
	}
	ends := [3][3]float64{
		{box.Min.X, box.Max.X, anchor.X},
		{box.Min.Y, box.Max.Y, anchor.Y},
		{box.Min.Z, box.Max.Z, anchor.Z},
	}
	for axis, end := range ends {
		lo := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(end[0]), proofarith.FloatRat(end[2])))
		hi := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(end[1]), proofarith.FloatRat(end[2])))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		lo.Add(lo, proofarith.FloatRat(allow))
		if maxMesh[axis].Cmp(lo) > 0 {
			lo = maxMesh[axis]
		}
		extent[axis] = lo
	}
	return extent, nil
}

// NonNegativeFinite reports whether a nonnegative displacement is finite.
func NonNegativeFinite(value float64) bool {
	return !proofbound.IsNonFinite(value) && value >= 0
}

// MeshReadingsPositive proves row dominance over the published mass and inertia bounds.
func MeshReadingsPositive(mass proofbound.BoundedScalar, diagonal, off [3]proofbound.BoundedScalar) bool {
	if mass.Value <= mass.Bound {
		return false
	}
	var offUpper [3]*big.Rat
	for i, v := range off {
		magnitude := proofarith.FloatRat(v.Value)
		magnitude.Abs(magnitude)
		offUpper[i] = new(big.Rat).Add(magnitude, proofarith.FloatRat(v.Bound))
	}
	for i, v := range diagonal {
		lower := new(big.Rat).Sub(proofarith.FloatRat(v.Value), proofarith.FloatRat(v.Bound))
		switch i {
		case 0:
			lower.Sub(lower, offUpper[0]).Sub(lower, offUpper[1])
		case 1:
			lower.Sub(lower, offUpper[0]).Sub(lower, offUpper[2])
		case 2:
			lower.Sub(lower, offUpper[1]).Sub(lower, offUpper[2])
		}
		if lower.Sign() <= 0 {
			return false
		}
	}
	return true
}
