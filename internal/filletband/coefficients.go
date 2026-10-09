package filletband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Coefficients is Table CF's strip polynomial (docs/loop-fillet-design.md
// §5.1): the strip S(t) between ℓ and ℓ offset t into F's material has area
// A1·t + A2·t² and first moment M1·t + M2·t² + M3·t³ about F's frame origin.
type Coefficients struct {
	A1, A2     Interval
	M1, M2, M3 Vec2
}

func zeroCoefficients() Coefficients {
	return Coefficients{A1: pointInt(0), A2: pointInt(0), M1: zeroVec2(), M2: zeroVec2(), M3: zeroVec2()}
}

func (c Coefficients) add(o Coefficients) Coefficients {
	return Coefficients{
		A1: proofbound.IntervalAdd(c.A1, o.A1), A2: proofbound.IntervalAdd(c.A2, o.A2),
		M1: c.M1.add(o.M1), M2: c.M2.add(o.M2), M3: c.M3.add(o.M3),
	}
}

// PieceKind names the patch a piece of ℓ carries (Table LF).
type PieceKind int

const (
	// Cylinder is LF1, a straight walk's quarter pipe.
	Cylinder PieceKind = iota + 1
	// InnerTorus is LF2, a circular walk with the material inside its circle.
	InnerTorus
	// OuterTorus is LF3, a circular walk with the material outside its circle.
	OuterTorus
	// HornTorus is LF6, the horn torus at a reflex corner.
	HornTorus
)

// Piece is one patch of the band in the band's own order — walk i's patch,
// then the LF6 patch at the corner after walk i — beside the readings Table CF
// and §5.3's patch areas take from it.
//
// A straight walk (Cylinder) states its Start, Length, unit direction Tau and
// inward normal Nu, and the end rates Kappa0 and Kappa1, cot(θ/2) at an LF4
// end and zero otherwise. A circular walk states its Center, Radius and
// Sweep, and Window, the exact radial directions of its two ends, lowest
// angle first; a whole circle sets WholeTurn. A reflex corner (HornTorus)
// states its corner Vertex, its turn Sweep, and Window, the exact inward
// normals of the leaving and the arriving walk. Walk and Corner index the
// walk or the corner the piece belongs to, −1 where it has none.
type Piece struct {
	Kind           PieceKind
	Walk, Corner   int
	Length, Radius Interval
	Kappa0, Kappa1 Interval
	Sweep          Interval
	Tau, Nu        Vec2
	Window         [2]ratVec
	WholeTurn      bool
	Start, Center  Point
	Vertex         Point
	Coefficients   Coefficients
}

// Pieces resolves ℓ into its patches in the band's order, each with its
// Table CF row. An LF5 or LF4 corner adds no piece of its own: its share is in
// its walks' end rates.
func (l Loop) Pieces() ([]Piece, error) {
	n := len(l.Walks)
	selected := make([]bool, n)
	for i := range selected {
		selected[i] = true
	}
	return l.PiecesSelected(selected)
}

// PiecesSelected measures only the selected walks of a loop. At a boundary
// between a selected straight walk and an unselected straight walk, the
// selected offset meets the unselected carrier. Its signed end rate is
// -cot(theta), where theta is the turn between the two walks. A corner
// between two selected walks keeps the ordinary loop-fillet class.
func (l Loop) PiecesSelected(selected []bool) ([]Piece, error) {
	n := len(l.Walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: a loop fillet's loop holds no walks`, decaderr.ErrDegenerate)
	}
	if len(selected) != n {
		return nil, fmt.Errorf(`%w: a loop fillet selects %d of %d walks`, decaderr.ErrDegenerate, len(selected), n)
	}
	if l.WholeTurn() {
		if !selected[0] {
			return nil, fmt.Errorf(`%w: a loop fillet selects no walks`, decaderr.ErrDegenerate)
		}
		p, err := circularPiece(l.Walks[0], 0)
		if err != nil {
			return nil, err
		}
		return []Piece{p}, nil
	}
	if len(l.Corners) != n {
		return nil, fmt.Errorf(`%w: a loop fillet's loop of %d walks classifies %d corners`, decaderr.ErrUnsupported, n, len(l.Corners))
	}
	kappa := make([]Interval, n)
	chosen := 0
	for k, c := range l.Corners {
		kappa[k] = pointInt(0)
		prev := (k + n - 1) % n
		if selected[prev] != selected[k] {
			if l.Walks[prev].Circular || l.Walks[k].Circular {
				return nil, fmt.Errorf(`%w: an open fillet chain ends beside a circular walk`, decaderr.ErrUnsupported)
			}
			a, err := direction(l.Walks[prev])
			if err != nil {
				return nil, err
			}
			b, err := direction(l.Walks[k])
			if err != nil {
				return nil, err
			}
			cross := a.cross(b)
			if cross.Sign() == 0 {
				return nil, fmt.Errorf(`%w: an open fillet chain ends at a parallel join`, decaderr.ErrUnsupported)
			}
			kappa[k] = point(new(big.Rat).Quo(new(big.Rat).Neg(a.dot(b)), cross))
			continue
		}
		if !selected[k] || c != Miter {
			continue
		}
		kv, err := Kappa(l.Walks[prev], l.Walks[k])
		if err != nil {
			return nil, err
		}
		kappa[k] = kv
	}
	var out []Piece
	for i, w := range l.Walks {
		if !selected[i] {
			continue
		}
		chosen++
		var p Piece
		var err error
		if w.Circular {
			p, err = circularPiece(w, i)
		} else {
			p, err = straightPiece(w, i, kappa[i], kappa[(i+1)%n])
		}
		if err != nil {
			return nil, err
		}
		out = append(out, p)
		k := (i + 1) % n
		if !selected[k] || l.Corners[k] != Reflex {
			continue
		}
		rp, err := reflexPiece(l.Walks[i], l.Walks[k], k)
		if err != nil {
			return nil, err
		}
		out = append(out, rp)
	}
	if chosen == 0 {
		return nil, fmt.Errorf(`%w: a loop fillet selects no walks`, decaderr.ErrDegenerate)
	}
	return out, nil
}

// Coefficients sums Table CF over ℓ's pieces.
func (l Loop) Coefficients() (Coefficients, error) {
	pieces, err := l.Pieces()
	if err != nil {
		return Coefficients{}, err
	}
	return SumCoefficients(pieces), nil
}

// SumCoefficients sums Table CF over pieces.
func SumCoefficients(pieces []Piece) Coefficients {
	out := zeroCoefficients()
	for _, p := range pieces {
		out = out.add(p.Coefficients)
	}
	return out
}

// straightPiece is CF1 for the straight walk w from P to Q with end rates k0
// at P and k1 at Q: the trapezoid {P + s·τ + w·ν : 0 ≤ w ≤ t, k0·w ≤ s ≤ ℓ −
// k1·w}. ℓ·τ = Q − P and ℓ·ν is Q − P turned left, so only m₃ reads the
// enclosed unit vectors.
func straightPiece(w Walk, i int, k0, k1 Interval) (Piece, error) {
	p, err := ratPoint(w.Start)
	if err != nil {
		return Piece{}, err
	}
	q, err := ratPoint(w.End)
	if err != nil {
		return Piece{}, err
	}
	d := q.sub(p)
	length, err := sqrtOf(d.dot(d))
	if err != nil {
		return Piece{}, err
	}
	tau, err := unit(d)
	if err != nil {
		return Piece{}, err
	}
	nu := Vec2{proofbound.IntervalNeg(tau[1]), tau[0]}
	ksum := proofbound.IntervalAdd(k0, k1)
	half := big.NewRat(1, 2)
	mid := ratVec{new(big.Rat).Mul(new(big.Rat).Add(p[0], q[0]), half), new(big.Rat).Mul(new(big.Rat).Add(p[1], q[1]), half)}
	perp := d.perp()
	var m2 Vec2
	for c := range m2 {
		m2[c] = proofbound.IntervalAdd(
			proofbound.IntervalAdd(
				proofbound.IntervalScale(ksum, new(big.Rat).Neg(new(big.Rat).Mul(p[c], half))),
				proofbound.IntervalScale(k1, new(big.Rat).Neg(new(big.Rat).Mul(d[c], half)))),
			point(new(big.Rat).Mul(perp[c], half)))
	}
	kdiff := proofbound.IntervalSub(proofbound.IntervalSquare(k1), proofbound.IntervalSquare(k0))
	m3 := tau.mul(scale(kdiff, 1, 6)).add(nu.mul(scale(ksum, -1, 3)))
	return Piece{
		Kind: Cylinder, Walk: i, Corner: -1, Length: length, Kappa0: k0, Kappa1: k1, Tau: tau, Nu: nu,
		Start: w.Start,
		Coefficients: Coefficients{
			A1: length,
			A2: scale(ksum, -1, 2),
			M1: ratVec2(mid).mul(length),
			M2: m2,
			M3: m3,
		},
	}, nil
}

// circularPiece is CF2 (material inside the circle, a counter-clockwise walk)
// or CF3 (material outside, a clockwise one) for the circular walk w: the
// annular sector of sweep β between radius R and R ∓ t. E is ∫(cos α, sin α)
// dα over the walk's angular interval taken increasing, read off its two
// exact ends: sin α = (P_v − C_v)/|P − C|. A whole circle has β = 2π and
// E = 0.
func circularPiece(w Walk, i int) (Piece, error) {
	c, err := ratPoint(w.Center)
	if err != nil {
		return Piece{}, err
	}
	kind, sign := InnerTorus, int64(-1)
	if !w.CCW {
		kind, sign = OuterTorus, 1
	}
	piece := Piece{Kind: kind, Walk: i, Corner: -1, Center: w.Center, WholeTurn: w.Closed}
	var radius, beta Interval
	e := zeroVec2()
	if w.Closed {
		rq, err := ratOf(w.Radius)
		if err != nil {
			return Piece{}, err
		}
		if rq.Sign() <= 0 {
			return Piece{}, fmt.Errorf(`%w: a loop fillet's circle has no radius`, decaderr.ErrDegenerate)
		}
		radius, beta = point(rq), proofbound.TwoPiInterval()
	} else {
		start, err := ratPoint(w.Start)
		if err != nil {
			return Piece{}, err
		}
		end, err := ratPoint(w.End)
		if err != nil {
			return Piece{}, err
		}
		lo, hi := start.sub(c), end.sub(c)
		if !w.CCW {
			lo, hi = hi, lo
		}
		piece.Window = [2]ratVec{lo, hi}
		ds := start.sub(c)
		radius, err = sqrtOf(ds.dot(ds))
		if err != nil {
			return Piece{}, err
		}
		ulo, err := unit(lo)
		if err != nil {
			return Piece{}, err
		}
		uhi, err := unit(hi)
		if err != nil {
			return Piece{}, err
		}
		e = Vec2{proofbound.IntervalSub(uhi[1], ulo[1]), proofbound.IntervalSub(ulo[0], uhi[0])}
		beta, err = sweepBetween(lo, hi)
		if err != nil {
			return Piece{}, err
		}
	}
	piece.Radius, piece.Sweep = radius, beta
	br := proofbound.IntervalMul(beta, radius)
	cv := ratVec2(c)
	piece.Coefficients = Coefficients{
		A1: br,
		A2: scale(beta, sign, 2),
		M1: cv.mul(br).add(e.mul(proofbound.IntervalSquare(radius))),
		M2: cv.mul(scale(beta, sign, 2)).add(e.mul(radius).scale(sign, 1)),
		M3: e.scale(1, 3),
	}
	return piece, nil
}

// sweepBetween encloses the counter-clockwise angle from lo to hi, in
// (0, 2π): atan2 of their exact cross and dot products, a full turn added
// where the cross product is negative.
func sweepBetween(lo, hi ratVec) (Interval, error) {
	cr, dt := lo.cross(hi), lo.dot(hi)
	if cr.Sign() == 0 && dt.Sign() >= 0 {
		return Interval{}, fmt.Errorf(`%w: a loop fillet's arc sweeps no angle a partial arc states`, decaderr.ErrUnsupported)
	}
	a := proofbound.Atan2Interval(cr, dt, false)
	if cr.Sign() < 0 {
		a = proofbound.IntervalAdd(a, proofbound.TwoPiInterval())
	}
	return a, nil
}

// reflexPiece is CF4 at the reflex corner where prev arrives at cur: the
// sector of radius t about the corner V filling the gap the two walls' strips
// leave, from the leaving walk's inward normal to the arriving walk's, of
// turn ψ below a half turn.
func reflexPiece(prev, cur Walk, k int) (Piece, error) {
	arr, err := inward(prev, true)
	if err != nil {
		return Piece{}, err
	}
	leave, err := inward(cur, false)
	if err != nil {
		return Piece{}, err
	}
	cr := arr.cross(leave)
	if cr.Sign() >= 0 {
		return Piece{}, fmt.Errorf(`%w: an LF6 corner turns right`, decaderr.ErrUnsupported)
	}
	psi := proofbound.Atan2Interval(new(big.Rat).Neg(cr), arr.dot(leave), false)
	ua, err := unit(arr)
	if err != nil {
		return Piece{}, err
	}
	ul, err := unit(leave)
	if err != nil {
		return Piece{}, err
	}
	v, err := ratPoint(cur.Start)
	if err != nil {
		return Piece{}, err
	}
	ev := Vec2{proofbound.IntervalSub(ua[1], ul[1]), proofbound.IntervalSub(ul[0], ua[0])}
	halfPsi := scale(psi, 1, 2)
	return Piece{
		Kind: HornTorus, Walk: -1, Corner: k, Sweep: psi, Vertex: cur.Start,
		Window: [2]ratVec{leave, arr},
		Coefficients: Coefficients{
			A1: pointInt(0),
			A2: halfPsi,
			M1: zeroVec2(),
			M2: ratVec2(v).mul(halfPsi),
			M3: ev.scale(1, 3),
		},
	}, nil
}
