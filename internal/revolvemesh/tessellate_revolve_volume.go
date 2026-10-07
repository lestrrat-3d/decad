package revolvemesh

import (
	"fmt"
	"math/big"
	"sync"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// RevolveAngularIntegralSteps is the fixed certified-subdivision budget §11's
// angular homotopy integral spends in the u direction.
//
// It is a separate constant from RevolveArcIntegralSteps because it is spent on
// a different scale: an Ecell reading is taken per CELL, while this one is taken
// ONCE for the whole mesh (RevolveAngularHomotopyFactor's own doc comment says
// why), so a deeper cut costs nothing per facet and the budget is spent where it
// buys the most. Like that one it is fixed rather than adaptive, and for the
// same reason: every piece's contribution is already an upper bound at any
// depth, so depth buys tightness and never soundness.
const RevolveAngularIntegralSteps = 64

// RevolveAngularHomotopyFactor proves the ANGULAR factor of
// docs/tessellation-design.md §11's per-cell triple integral: the part of
//
//	Icell ≥ ∫_[0,1]³ |∂H/∂λ · (∂H/∂t × ∂H/∂u)| d(λ, t, u)
//
// that survives once the meridian direction has been integrated out. It depends
// on the angular step dφ alone, so one reading answers for every cell of the
// mesh and for every angular interval of each of them.
//
// The separation is exact, not an approximation. Write the homotopy §11 states
// in the axis basis (w, e0, e1), with e(φ) = cos φ·e0 + sin φ·e1, A(u) =
// e(φ0 + u·dφ) the rotated point, B0 = e(φ0), B1 = e(φ1) and C(u) =
// (1−u)·B0 + u·B1 the chord point:
//
//	H(λ, t, u) = a3 + z(t)·w + ρ(t)·((1−λ)·A(u) + λ·C(u))
//
// with z and ρ AFFINE in t along the meridian chord. Then ∂H/∂λ = ρ·(C − A) and
// ∂H/∂u = ρ·∂E/∂u are both in the (e0, e1) plane while ∂H/∂t = z'·w + ρ'·E, so
// the ρ' term of the cross product points along w and the ∂H/∂λ dot kills it.
// What is left is
//
//	∂H/∂λ · (∂H/∂t × ∂H/∂u) = z' · ρ(t)² · (Eu ∧ (C − A))
//
// where ∧ is the in-plane scalar cross product and Eu = ∂E/∂u. The t direction
// is therefore a closed-form rational factor the caller owns
// (RevolveCellSweptVolume), and everything else is this function's:
//
//	Eu ∧ (C − A) = (1−λ)·P(u) + λ·Q(u)
//	P(u) = dφ·((1−u)·(1 − cos(u·dφ)) + u·(1 − cos((1−u)·dφ)))
//	Q(u) = sin(u·dφ)·(1 − cos dφ) − sin(dφ)·(1 − cos(u·dφ))
//
// P is written in that grouped form deliberately: the raw expression
// 1 − (1−u)cos(u·dφ) − u·cos((1−u)·dφ) is a difference of order-one terms whose
// value is of order dφ², and the grouping makes every term non-negative and the
// cancellation vanish. Both are ODD in dφ, so |·| is unchanged by the sweep
// sense and the reading is taken at |dφ|.
//
// The λ direction then integrates in closed form and exactly, by the triangle
// inequality on a CONVEX COMBINATION:
//
//	∫₀¹ |(1−λ)·P + λ·Q| dλ ≤ ∫₀¹ ((1−λ)·|P| + λ·|Q|) dλ = (|P| + |Q|) / 2
//
// so the whole reading is (∫|P| + ∫|Q|)/2. Neither of those two has a closed
// form — the zeros are arcsines — so each takes
// docs/tessellation-design.md §15's other admissible path, CERTIFIED INTERVAL
// SUBDIVISION: [0, 1] is cut into RevolveAngularIntegralSteps equal pieces, both
// functions are enclosed at every NODE of that cut, and one piece is charged the
// larger of its two nodes' magnitudes plus the proven second-order allowance
// AngularHomotopyBulges states. Because the absolute value is taken inside every
// piece, nothing cancels between pieces and the sum is an upper bound at any
// depth.
//
// The nodes are read, rather than a whole piece being enclosed at once, for the
// reason RevArcCell.rhoNodes gives for its own subdivision, and the reason
// binds harder here: P is of order dφ³ while a piece-wide RadSinCosSpan widens
// by the piece's own width, of order dφ/N. A whole-piece enclosure is therefore
// not merely loose, it stops converging — the node reading plus a second-order
// term is what keeps a fixed budget worth spending.
//
// Nothing here calls math.Sin or math.Cos, and nothing here compares against π:
// RadSinCosSpan reduces through internal/proofbound/moments_trig.go's own certified series.
//
// The reading is a pure function of the exact step, and a suite that
// tessellates many revolves on one angular plan asks for the same step again
// and again, so readings are memoised process-wide on the step's exact
// endpoints (revolveHomotopyMemo). Every call returns a fresh *big.Rat the
// caller may mutate; the memo never hands out the value it holds.
func RevolveAngularHomotopyFactor(step proofbound.RatInterval) (*big.Rat, error) {
	key := step.Lo.RatString() + "|" + step.Hi.RatString()
	if e, ok := revolveHomotopyMemo.get(key); ok {
		if e.err != nil {
			return nil, e.err
		}
		return e.val, nil
	}
	v, err := revolveAngularHomotopyFactorUncached(step)
	revolveHomotopyMemo.put(key, v, err)
	if err != nil {
		return nil, err
	}
	return new(big.Rat).Set(v), nil
}

// revolveHomotopyMemoCap bounds the memo. A process sees one step per angular
// plan (nPhi and sweep) in use — the whole apitest suite asks for about 80
// distinct ones — so the cap is not reached in practice; it exists only so an
// unusual workload cannot grow the map without limit. An entry holds a reading
// of under a kilobyte.
const revolveHomotopyMemoCap = 256

// homotopyMemo is RevolveAngularHomotopyFactor's bounded process-wide memo.
// A full map is cleared before the next insert: plain eviction, since the
// working set is far below the cap.
type homotopyMemo struct {
	mu      sync.Mutex
	entries map[string]homotopyMemoEntry
}

type homotopyMemoEntry struct {
	val *big.Rat
	err error
}

var revolveHomotopyMemo = &homotopyMemo{entries: map[string]homotopyMemoEntry{}}

// get returns the memoised reading for key, its value a fresh copy the caller
// owns, or ok false when none is held.
func (m *homotopyMemo) get(key string) (homotopyMemoEntry, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	e, ok := m.entries[key]
	if !ok || e.err != nil {
		return e, ok
	}
	return homotopyMemoEntry{val: new(big.Rat).Set(e.val)}, true
}

// put records the reading for key. It takes ownership of val: the caller must
// not hand val itself to anyone afterwards.
func (m *homotopyMemo) put(key string, val *big.Rat, err error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.entries) >= revolveHomotopyMemoCap {
		clear(m.entries)
	}
	m.entries[key] = homotopyMemoEntry{val: val, err: err}
}

// revolveAngularHomotopyFactorUncached is RevolveAngularHomotopyFactor's
// reading without the memo; its doc comment states the argument.
func revolveAngularHomotopyFactorUncached(step proofbound.RatInterval) (*big.Rat, error) {
	d := IntervalAbsSpan(step)
	if d.Lo.Sign() < 0 || d.Hi.Cmp(d.Lo) < 0 {
		return nil, ErrRevolveAngularHomotopy
	}
	one := proofbound.PointInterval(big.NewRat(1, 1))
	sinD, cosD, ok := proofbound.RadSinCosSpan(d)
	if !ok {
		return nil, ErrRevolveAngularHomotopy
	}
	versD := proofbound.IntervalSub(one, cosD)

	n := int64(RevolveAngularIntegralSteps)
	pAt := make([]*big.Rat, n+1)
	qAt := make([]*big.Rat, n+1)
	sinAt := make([]proofbound.RatInterval, n+1)
	cosAt := make([]proofbound.RatInterval, n+1)
	for i := int64(0); i <= n; i++ {
		u := big.NewRat(i, n)
		sin, cos, ok := proofbound.RadSinCosSpan(proofbound.IntervalScale(d, u))
		if !ok {
			return nil, ErrRevolveAngularHomotopy
		}
		sinAt[i], cosAt[i] = sin, cos
	}
	for i := int64(0); i <= n; i++ {
		u := big.NewRat(i, n)
		co := new(big.Rat).Sub(big.NewRat(1, 1), u)
		sinA, cosA, cosB := sinAt[i], cosAt[i], cosAt[n-i]
		versA := proofbound.IntervalSub(one, cosA)
		p := proofbound.IntervalMul(d, proofbound.IntervalAdd(
			proofbound.IntervalScale(versA, co),
			proofbound.IntervalScale(proofbound.IntervalSub(one, cosB), u),
		))
		q := proofbound.IntervalSub(proofbound.IntervalMul(sinA, versD), proofbound.IntervalMul(sinD, versA))
		pAt[i], qAt[i] = proofbound.IntervalAbsUpper(p), proofbound.IntervalAbsUpper(q)
	}

	bulgeP, bulgeQ := AngularHomotopyBulges(d, n)
	total := new(big.Rat)
	width := big.NewRat(1, n)
	for i := range n {
		piece := new(big.Rat).Add(survey2d.RatMax(pAt[i], pAt[i+1]), bulgeP)
		piece.Add(piece, new(big.Rat).Add(survey2d.RatMax(qAt[i], qAt[i+1]), bulgeQ))
		total.Add(total, new(big.Rat).Mul(piece, width))
	}
	return total.Mul(total, big.NewRat(1, 2)), nil
}

// AngularHomotopyBulges states the per-piece second-order allowance
// RevolveAngularHomotopyFactor charges beside each piece's two node readings,
// one for P and one for Q.
//
// The argument is elementary and needs no monotonicity: a twice-differentiable f
// departs from the chord through its own two endpoints by at most
// max|f”|·h²/8, so |f| over a piece of width h = 1/N is at most the larger of
// its endpoint magnitudes plus that. What is left is a bound on each second
// derivative, and both differentiate in closed form. Writing
// g(u) = (1−u)·(1 − cos(u·dφ)), so that P = dφ·(g(u) + g(1−u)):
//
//	g”(u) = −2·dφ·sin(u·dφ) + (1−u)·dφ²·cos(u·dφ)
//	Q”(u) = −dφ²·sin(u·dφ)·(1 − cos dφ) − dφ²·sin(dφ)·cos(u·dφ)
//
// With |cos| ≤ 1, |sin x| ≤ min(1, |x|) and |1 − cos x| ≤ min(2, x²/2) — the
// elementary bounds, not derived ones — and |u·dφ| ≤ dφ throughout:
//
//	|P”| ≤ 2·dφ·(2·dφ·S + dφ²)      |Q”| ≤ dφ²·S·(V + 1)
//
// for S = min(1, dφ) and V = min(2, dφ²/2). Both S bounds matter: taking S = 1
// alone would charge dφ² where the truth is dφ³, and the allowance would stop
// falling with the step while everything it sits beside kept falling.
func AngularHomotopyBulges(d proofbound.RatInterval, n int64) (*big.Rat, *big.Rat) {
	dh := new(big.Rat).Set(d.Hi)
	sq := new(big.Rat).Mul(dh, dh)
	sinB := survey2d.RatMin(big.NewRat(1, 1), dh)
	versB := survey2d.RatMin(big.NewRat(2, 1), new(big.Rat).Mul(sq, big.NewRat(1, 2)))

	p := new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), dh), new(big.Rat).Add(
		new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), dh), sinB),
		sq,
	))
	q := new(big.Rat).Mul(new(big.Rat).Mul(sq, sinB), new(big.Rat).Add(versB, big.NewRat(1, 1)))
	denom := new(big.Rat).SetInt64(8 * n * n)
	return new(big.Rat).Quo(p, denom), new(big.Rat).Quo(q, denom)
}

var ErrRevolveAngularHomotopy = fmt.Errorf(`%w: a revolve cell's angular homotopy states no enclosure of the volume it sweeps, so this mesh can prove no occupied-volume bound`, decaderr.ErrUnsupported)

// RevolveCellSweptVolume is docs/tessellation-design.md §11's Icell for ONE
// meridian cell at ONE angular interval: the meridian factor of the triple
// integral RevolveAngularHomotopyFactor separated out, times that shared
// angular factor.
//
// With z and ρ affine along the meridian chord, the integrand's t dependence is
// |z'|·ρ(t)² and both pieces are exact rationals in the cell's own enclosed
// endpoints:
//
//	∫₀¹ ρ(t)² dt = (ρ0² + ρ0·ρ1 + ρ1²) / 3
//
// A cell with one ring ON the axis needs no separate statement: ρ0 = 0 makes
// H's λ = 1 surface the fan triangle the mesh actually holds, exactly as the
// λ = 1 surface of an off-axis cell is its planar quad — the four corners of
// that quad are coplanar (their triple product carries a D ∧ D factor), so the
// bilinear patch H(1, ·, ·) IS the two held facets and no fourth stage exists
// between them.
//
// The enclosures are read rather than the stored floats because §11's homotopy
// runs between IDEAL-coordinate surfaces; what the stored floats cost is
// Mconstruct's and Mround's business, one stage later.
func RevolveCellSweptVolume(lo, hi RevMeridian, angular *big.Rat) *big.Rat {
	third := big.NewRat(1, 3)
	quad := proofbound.IntervalScale(proofbound.IntervalAdd(
		proofbound.IntervalAdd(proofbound.IntervalSquare(lo.RhoIv), proofbound.IntervalSquare(hi.RhoIv)),
		proofbound.IntervalMul(lo.RhoIv, hi.RhoIv),
	), third)
	axial := proofbound.IntervalAbsUpper(proofbound.IntervalSub(hi.ZIv, lo.ZIv))
	return new(big.Rat).Mul(new(big.Rat).Mul(axial, proofbound.IntervalAbsUpper(quad)), angular)
}
