package coilshell

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// World is the record's world frame under the placement, each vector an
// exact rational: a point (x, y, z) of plane coordinates sits at
// O + x·U + y·V + z·N.
type World struct {
	o, u, v, n sweeparc.RatVec
}

func worldOf(frame r3.Frame, xform r3.Transform) World {
	w := World{
		o: sweeparc.VecOf(frame.Origin()),
		u: sweeparc.VecOf(frame.U()),
		v: sweeparc.VecOf(frame.V()),
		n: sweeparc.VecOf(frame.N()),
	}
	if xform == r3.Identity() {
		return w
	}
	basis := xform.Basis()
	ex, ey, ez := sweeparc.VecOf(basis.EX), sweeparc.VecOf(basis.EY), sweeparc.VecOf(basis.EZ)
	linear := func(p sweeparc.RatVec) sweeparc.RatVec {
		return sweeparc.Add(sweeparc.Scale(ex, p[0]), sweeparc.Scale(ey, p[1]), sweeparc.Scale(ez, p[2]))
	}
	w.o = sweeparc.Add(linear(w.o), sweeparc.VecOf(xform.Translation()))
	w.u, w.v, w.n = linear(w.u), linear(w.v), linear(w.n)
	return w
}

// affine reads the denoted map L = [U V N] off the exact world vectors: its
// determinant, exact, and its orthonormality defect e, the entrywise absolute
// sum of LᵀL − I. Every eigenvalue of LᵀL lies in [1 − e, 1 + e], so L
// scales a length by a factor in [1 − e, 1 + e] (for e ≤ 1) and an area by
// one in the same range; an exactly orthonormal map answers det 1 and e 0.
func (w World) affine() (*big.Rat, *big.Rat) {
	var m [3][3]*big.Rat
	for k := range 3 {
		m[k] = [3]*big.Rat{w.u[k], w.v[k], w.n[k]}
	}
	return sweeparc.Dot(w.u, sweeparc.Cross(w.v, w.n)), massmoment.OrthonormalityDefect(m)
}

// Point lifts plane coordinates given as intervals.
func (w World) Point(x, y, z coil.Iv) proofbound.IvVec3 {
	var out proofbound.IvVec3
	for axis := range 3 {
		c := coil.Point(w.o[axis])
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(x, w.u[axis]))
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(y, w.v[axis]))
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(z, w.n[axis]))
		out[axis] = c
	}
	return out
}

// Vector lifts a plane-coordinate direction (no origin).
func (w World) Vector(x, y, z coil.Iv) proofbound.IvVec3 {
	var out proofbound.IvVec3
	for axis := range 3 {
		c := proofbound.IntervalScale(x, w.u[axis])
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(y, w.v[axis]))
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(z, w.n[axis]))
		out[axis] = c
	}
	return out
}

// Record is the record's exact reading: the profile polygon, every
// vertex's axis coordinates and the axis itself, shared by the held table and
// the readings.
type Record struct {
	Pts             []sectionrecord.Point2
	LoopIdx         [][]int
	U, V            []*big.Rat
	Rho, Zeta       []coil.Iv
	Axis            coil.Axis
	Pitch, Turns    *big.Rat
	Sigma           int
	N               int64
	World           World
	Det, Defect     *big.Rat
	Stretch         float64
	StationsPerTurn int64
}

// Stretched widens a positive plane-coordinate area or length enclosure by
// the denoted map's defect. An exactly orthonormal map returns x unchanged.
func (rec Record) Stretched(x coil.Iv) coil.Iv {
	if rec.Defect.Sign() == 0 {
		return x
	}
	one := big.NewRat(1, 1)
	lo := new(big.Rat).Mul(x.Lo, new(big.Rat).Sub(one, rec.Defect))
	hi := new(big.Rat).Mul(x.Hi, new(big.Rat).Add(one, rec.Defect))
	return proofbound.Interval(lo, hi)
}

// Volume scales the plane-coordinate coil volume by the placed frame's exact determinant.
func Volume(rec Record, m coil.Moments) coil.Iv {
	return proofbound.IntervalScale(coil.Volume(m, rec.Turns), new(big.Rat).Abs(rec.Det))
}

// Input is the recorded profile and measured sweep selected by the root payload.
type Input struct {
	Profile         momentinput.Profile
	Axis            AxisInput
	Frame           r3.Frame
	Transform       r3.Transform
	Pitch, Turns    float64
	LeftHand        bool
	StationsPerTurn int64
	MaxStations     int64
}

// AxisInput holds the measured point and direction of the profile-plane axis.
type AxisInput struct {
	AU, AV, AUBound, AVBound float64
	DU, DV, DUBound, DVBound float64
	Side                     int
}

func axisOf(in AxisInput) (coil.Axis, error) {
	aU, ok1 := coil.Measured(in.AU, in.AUBound)
	aV, ok2 := coil.Measured(in.AV, in.AVBound)
	dU, ok3 := coil.Measured(in.DU, in.DUBound)
	dV, ok4 := coil.Measured(in.DV, in.DVBound)
	if !ok1 || !ok2 || !ok3 || !ok4 {
		return coil.Axis{}, fmt.Errorf(`%w: the coil axis carries a non-finite reading`, decaderr.ErrUnsupported)
	}
	return coil.Axis{AU: aU, AV: aV, DU: dU, DV: dV, Side: in.Side}, nil
}

// Read resolves the recorded coil into exact section and placement inputs.
func Read(in Input) (Record, error) {
	pts, loopIdx, err := coil.Loops(in.Profile.Outer, in.Profile.Holes)
	if err != nil {
		return Record{}, err
	}
	axis, err := axisOf(in.Axis)
	if err != nil {
		return Record{}, err
	}
	rec := Record{
		Pts: pts, LoopIdx: loopIdx, Axis: axis,
		Pitch: proofarith.FloatRat(in.Pitch), Turns: proofarith.FloatRat(in.Turns), Sigma: 1,
		World: worldOf(in.Frame, in.Transform), StationsPerTurn: in.StationsPerTurn,
	}
	if in.LeftHand {
		rec.Sigma = -1
	}
	rec.Det, rec.Defect = rec.World.affine()
	if rec.Defect.Cmp(big.NewRat(1, 2)) >= 0 {
		return Record{}, fmt.Errorf(`%w: the coil's placed frame departs from orthonormal by %s`, decaderr.ErrUnsupported, rec.Defect.FloatString(3))
	}
	rec.Stretch = proofbound.RatFloatUp(new(big.Rat).Add(big.NewRat(1, 1), rec.Defect))
	n, ok := coil.StationCount(rec.Turns, in.StationsPerTurn)
	if !ok || n > in.MaxStations {
		return Record{}, fmt.Errorf(`%w: the coil's station count exceeds its ceiling`, decaderr.ErrUnsupported)
	}
	rec.N = n
	rec.U = make([]*big.Rat, len(pts))
	rec.V = make([]*big.Rat, len(pts))
	rec.Rho = make([]coil.Iv, len(pts))
	rec.Zeta = make([]coil.Iv, len(pts))
	for i, p := range pts {
		rec.U[i], rec.V[i] = proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
		rec.Rho[i], rec.Zeta[i] = axis.Coords(rec.U[i], rec.V[i])
		if rec.Rho[i].Lo.Sign() <= 0 {
			return Record{}, fmt.Errorf(`%w: coil profile vertex %d is not proven off the axis`, decaderr.ErrUnsupported, i)
		}
	}
	return rec, nil
}

// Shell is §5's held shell: the vertex table, each vertex's β and the
// largest station rounding, the oriented triangle set and the bookkeeping
// Table CB's topology reads. Vertex v of station j is index j·stride + v.
// Wall triangles come first, two per cell in (station, loop, segment) order;
// wallOf[k] is the profile vertex starting wall triangle k's segment.
type Shell struct {
	Verts         []r3.Vec
	VertexBound   []float64
	MaxRound      float64
	Delta         float64
	Tris          [][3]int
	WallOf        []int
	Walls         int
	CapStartCount int
	Reversed      bool
	Stride        int
}

// At returns vertex v at station j in the held vertex table.
func (s Shell) At(j int64, v int) int { return int(j)*s.Stride + v }

// Build is §5.2–§5.5: stations, held vertices, triangles, the one
// orientation decision, β and δ, and the crossing audit.
func Build(ctx context.Context, rec Record) (Shell, error) {
	stride := len(rec.Pts)
	n := rec.N
	sh := Shell{Stride: stride}

	// The per-vertex terms of the station formula, lifted once as exact
	// intervals and held as bounded floats:
	// X(v, j) = P_v + (cos θ_j − 1)·B_v + t_j·D + sin θ_j·S_v.
	// Each station point is then evaluated in float arithmetic whose bound
	// charges every operand's own bound and every product's and sum's exact
	// rounding (coil.Mul, coil.Add), so the held coordinate and its round are
	// proven without an interval evaluation per vertex. Station 0 reads P_v
	// alone: every other term is an exact zero there.
	erU, erV := rec.Axis.Radial()
	zero := coil.Point(new(big.Rat))
	held := func(x coil.Iv) (proofbound.BoundedScalar, error) {
		b, ok := coil.Held(x)
		if !ok {
			return b, fmt.Errorf(`%w: a coil station term runs past the representable float64 range`, decaderr.ErrUnsupported)
		}
		return b, nil
	}
	pv := make([][3]proofbound.BoundedScalar, stride)
	bv := make([][3]proofbound.BoundedScalar, stride)
	sv := make([][3]proofbound.BoundedScalar, stride)
	tilt := big.NewRat(int64(rec.Sigma*rec.Axis.Side), 1)
	for v := range stride {
		p := rec.World.Point(coil.Point(rec.U[v]), coil.Point(rec.V[v]), zero)
		b := rec.World.Vector(proofbound.IntervalMul(rec.Rho[v], erU), proofbound.IntervalMul(rec.Rho[v], erV), zero)
		q := rec.World.Vector(zero, zero, proofbound.IntervalScale(rec.Rho[v], tilt))
		for axis := range 3 {
			var err error
			if pv[v][axis], err = held(p[axis]); err != nil {
				return Shell{}, err
			}
			if bv[v][axis], err = held(b[axis]); err != nil {
				return Shell{}, err
			}
			if sv[v][axis], err = held(q[axis]); err != nil {
				return Shell{}, err
			}
		}
	}
	pitchIv := coil.Point(rec.Pitch)
	dvec := rec.World.Vector(proofbound.IntervalMul(pitchIv, rec.Axis.DU), proofbound.IntervalMul(pitchIv, rec.Axis.DV), zero)
	one := coil.Point(big.NewRat(1, 1))

	total := int(n+1) * stride
	sh.Verts = make([]r3.Vec, total)
	round := make([]float64, total)
	for j := int64(0); j <= n; j++ {
		if err := ctx.Err(); err != nil {
			return Shell{}, err
		}
		t := coil.Fraction(rec.Turns, j, n)
		sinT, cosT := coil.TurnSinCos(t)
		cm1, err := held(proofbound.IntervalSub(cosT, one))
		if err != nil {
			return Shell{}, err
		}
		sn, err := held(sinT)
		if err != nil {
			return Shell{}, err
		}
		var slide [3]proofbound.BoundedScalar
		for axis := range 3 {
			if slide[axis], err = held(proofbound.IntervalScale(dvec[axis], t)); err != nil {
				return Shell{}, err
			}
		}
		for v := range stride {
			var coords [3]float64
			worst := 0.0
			for axis := range 3 {
				c := coil.Add(pv[v][axis], coil.Mul(cm1, bv[v][axis]))
				c = coil.Add(c, slide[axis])
				c = coil.Add(c, coil.Mul(sn, sv[v][axis]))
				if proofbound.IsNonFinite(c.Value) || proofbound.IsNonFinite(c.Bound) {
					return Shell{}, fmt.Errorf(`%w: a coil station point runs past the representable float64 range`, decaderr.ErrUnsupported)
				}
				coords[axis] = c.Value
				worst = math.Max(worst, c.Bound)
			}
			k := sh.At(j, v)
			sh.Verts[k] = r3.NewVec(coords[0], coords[1], coords[2])
			round[k] = proofbound.Radius3D(worst)
			sh.MaxRound = math.Max(sh.MaxRound, round[k])
		}
	}

	// Walls, ring by ring, in the prism's lateral order; then the caps from
	// one triangulation of the recorded region, capStart reversed and capEnd
	// retained (docs/loft-design.md §5's cap seeding).
	for j := range n {
		for _, idx := range rec.LoopIdx {
			m := len(idx)
			for k := range m {
				v, w := idx[k], idx[(k+1)%m]
				b0, b1 := sh.At(j, v), sh.At(j, w)
				t0, t1 := sh.At(j+1, v), sh.At(j+1, w)
				sh.Tris = append(sh.Tris, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
				sh.WallOf = append(sh.WallOf, v, v)
			}
		}
	}
	sh.Walls = len(sh.Tris)
	capTris, err := triangulation.Triangulate(ctx, rec.Pts, rec.LoopIdx)
	if err != nil {
		return Shell{}, triangulation.WrapLoftError(err)
	}
	for _, t := range capTris {
		sh.Tris = append(sh.Tris, [3]int{sh.At(0, t[0]), sh.At(0, t[2]), sh.At(0, t[1])})
	}
	sh.CapStartCount = len(capTris)
	for _, t := range capTris {
		sh.Tris = append(sh.Tris, [3]int{sh.At(n, t[0]), sh.At(n, t[1]), sh.At(n, t[2])})
	}

	// One orientation decision for the whole shell: the exact signed
	// tetrahedron sum over the held floats.
	switch tessellation.OrientationSign(sh.Verts, sh.Tris, sh.Verts[0]) {
	case 0:
		return Shell{}, fmt.Errorf(`%w: the coil's held shell encloses no signed volume`, decaderr.ErrUnsupported)
	case -1:
		sh.Reversed = true
		for k, t := range sh.Tris {
			sh.Tris[k] = [3]int{t[0], t[2], t[1]}
		}
	}
	for k, t := range sh.Tris {
		if !triangleOpen(sh.Verts[t[0]], sh.Verts[t[1]], sh.Verts[t[2]]) && loftmesh.TriangleCollapsed(sh.Verts, t) {
			return Shell{}, fmt.Errorf(`%w: rounding the coil's station points collapsed its triangle %d`, decaderr.ErrUnsupported, k)
		}
	}

	// β (§5.4): every cell's departure — the analytic leg of its segment
	// (coil.CellDepartureUpper: sag, twist and shift under the shifted
	// correspondence, carried through L by its stretch) plus the cell's
	// largest corner rounding — charged to each of its four corners. The
	// analytic leg depends on the segment and the station step alone, so it
	// is read once per segment.
	sh.VertexBound = append([]float64(nil), round...)
	dt := new(big.Rat).Quo(rec.Turns, big.NewRat(n, 1))
	analytic := make([]float64, stride)
	for _, idx := range rec.LoopIdx {
		m := len(idx)
		for k := range m {
			v, w := idx[k], idx[(k+1)%m]
			dep, ok := coil.CellDepartureUpper(rec.Rho[v], rec.Rho[w], rec.Pitch, dt)
			if !ok {
				return Shell{}, fmt.Errorf(`%w: coil profile segment %d is not proven off the axis`, decaderr.ErrUnsupported, v)
			}
			analytic[v] = proofbound.ProductUpper(proofbound.RatFloatUp(dep), rec.Stretch)
			if proofbound.IsNonFinite(analytic[v]) {
				return Shell{}, fmt.Errorf(`%w: a coil wall cell states no finite departure`, decaderr.ErrUnsupported)
			}
		}
	}
	for j := range n {
		if err := ctx.Err(); err != nil {
			return Shell{}, err
		}
		for _, idx := range rec.LoopIdx {
			m := len(idx)
			for k := range m {
				v, w := idx[k], idx[(k+1)%m]
				corners := [4]int{sh.At(j, v), sh.At(j+1, v), sh.At(j, w), sh.At(j+1, w)}
				cellRound := 0.0
				for _, c := range corners {
					cellRound = math.Max(cellRound, round[c])
				}
				departure := proofbound.AbsSumUpper(analytic[v], cellRound)
				for _, c := range corners {
					sh.VertexBound[c] = math.Max(sh.VertexBound[c], departure)
				}
			}
		}
	}
	for _, b := range sh.VertexBound {
		sh.Delta = math.Max(sh.Delta, b)
	}

	if err := ctx.Err(); err != nil {
		return Shell{}, err
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), sh.Verts, sh.Tris); err != nil {
		var contact *loftmesh.LoftContactError
		if errors.As(err, &contact) {
			// CS9: CP5 already proved the true solid simple, so a held
			// crossing is a station artefact, never a defect of the solid.
			return Shell{}, fmt.Errorf(`%w: the coil's held triangles %d and %d %s; two turns pass closer than its %d stations per turn resolve`,
				decaderr.ErrUnsupported, contact.I, contact.J, contact.Reason, rec.StationsPerTurn)
		}
		return Shell{}, err
	}
	return sh, nil
}

// triangleOpen is a float certificate that a held triangle is NOT
// collapsed: one coordinate plane's 2D orientation determinant of its corners
// whose float value exceeds that evaluation's own error bound. The products
// are rounded by explicit conversions, so the bound below holds for the
// operations as written: each difference, each product and the final
// difference round once, and |error| ≤ 4u·(|l| + |r|) with u = 2⁻⁵³ is far
// inside the 1e-15 factor read here. A tiny sum, where underflow could break
// that relative bound, certifies nothing. False proves nothing; the caller
// then asks the exact test.
func triangleOpen(a, b, c r3.Vec) bool {
	pa, pb, pc := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}, [3]float64{c.X, c.Y, c.Z}
	for i := range 3 {
		j := (i + 1) % 3
		l := float64((pb[i] - pa[i]) * (pc[j] - pa[j]))
		r := float64((pb[j] - pa[j]) * (pc[i] - pa[i]))
		sum := math.Abs(l) + math.Abs(r)
		if !(sum > 0x1p-900) || proofbound.IsNonFinite(sum) {
			continue
		}
		if math.Abs(float64(l-r)) > 1e-15*sum {
			return true
		}
	}
	return false
}

// Held rounds an interval to its nearest float with the exact outward
// error, refusing a non-finite value (CS10).
func Held(x coil.Iv, what string) (float64, float64, error) {
	held, _ := coil.Mid(x).Float64()
	if math.IsNaN(held) || math.IsInf(held, 0) {
		return 0, 0, fmt.Errorf(`%w: the coil's %s is not finite`, decaderr.ErrUnsupported, what)
	}
	bound := proofbound.IntervalFloatError(x, held)
	if math.IsNaN(bound) || math.IsInf(bound, 0) {
		return 0, 0, fmt.Errorf(`%w: the coil's %s has no finite bound`, decaderr.ErrUnsupported, what)
	}
	return held, bound, nil
}
