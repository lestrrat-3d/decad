package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// This file builds docs/helix-design.md §5's held shell: the stations, the
// interval-enclosed station points rounded once into the held vertex table,
// each vertex's β, the triangle set, its one orientation decision and the
// crossing audit (Table CS rows CS9 and CS10).
//
// The coil the record denotes is taken in the plane frame's own coordinates
// (x, y) in the plane and z along its normal, with the frame's held U, V and
// N and the placement's held basis and translation read as exact rationals,
// the convention docs/loft-design.md §5's lift and the mitred sweep's
// placement already follow. In those coordinates the profile vertex p of axis
// coordinates (ρ, ζ) sits at station fraction t at
//
//	p + ρ·(cos 2πt − 1)·e_r + pitch·t·d,   z = σ·Side·ρ·sin 2πt
//
// because e_t = d × e_r is Side times the plane normal.

// coilWorld is the record's world frame under the placement, each vector an
// exact rational: a point (x, y, z) of plane coordinates sits at
// O + x·U + y·V + z·N.
type coilWorld struct {
	o, u, v, n sweeparc.RatVec
}

func coilWorldOf(frame r3.Frame, xform r3.Transform) coilWorld {
	w := coilWorld{
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

// point lifts plane coordinates given as intervals.
func (w coilWorld) point(x, y, z coil.Iv) proofbound.IvVec3 {
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

// vector lifts a plane-coordinate direction (no origin).
func (w coilWorld) vector(x, y, z coil.Iv) proofbound.IvVec3 {
	var out proofbound.IvVec3
	for axis := range 3 {
		c := proofbound.IntervalScale(x, w.u[axis])
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(y, w.v[axis]))
		c = proofbound.IntervalAdd(c, proofbound.IntervalScale(z, w.n[axis]))
		out[axis] = c
	}
	return out
}

// coilRecord is the record's exact reading: the profile polygon, every
// vertex's axis coordinates and the axis itself, shared by the held table and
// the readings.
type coilRecord struct {
	pts     []Point2
	loopIdx [][]int
	u, v    []*big.Rat
	rho     []coil.Iv
	zeta    []coil.Iv
	axis    coil.Axis
	pitch   *big.Rat
	turns   *big.Rat
	sigma   int
	n       int64
	world   coilWorld
}

func readCoilRecord(cp coilPayload) (coilRecord, error) {
	pts, loopIdx, err := coil.Loops(cp.profile.Outer, cp.profile.Holes)
	if err != nil {
		return coilRecord{}, err
	}
	axis, err := coilAxis(cp.line, cp.side)
	if err != nil {
		return coilRecord{}, err
	}
	rec := coilRecord{
		pts: pts, loopIdx: loopIdx, axis: axis,
		pitch: ratOf(cp.pitch), turns: ratOf(cp.turns), sigma: 1,
		world: coilWorldOf(cp.frame, cp.xform),
	}
	if cp.leftHand {
		rec.sigma = -1
	}
	n, ok := coil.StationCount(rec.turns, coilStationsPerTurn)
	if !ok || n > maxCoilStations {
		return coilRecord{}, fmt.Errorf(`%w: the coil's station count exceeds its ceiling`, ErrUnsupported)
	}
	rec.n = n
	rec.u = make([]*big.Rat, len(pts))
	rec.v = make([]*big.Rat, len(pts))
	rec.rho = make([]coil.Iv, len(pts))
	rec.zeta = make([]coil.Iv, len(pts))
	for i, p := range pts {
		rec.u[i], rec.v[i] = ratOf(p.U), ratOf(p.V)
		rec.rho[i], rec.zeta[i] = axis.Coords(rec.u[i], rec.v[i])
		if rec.rho[i].Lo.Sign() <= 0 {
			// CS5 proved the region off the axis from the boundary scan;
			// a vertex reading that disagrees is a contradiction this build
			// refuses rather than carries.
			return coilRecord{}, fmt.Errorf(`%w: coil profile vertex %d is not proven off the axis`, ErrUnsupported, i)
		}
	}
	return rec, nil
}

// coilShell is §5's held shell: the vertex table, each vertex's β and the
// largest station rounding, the oriented triangle set and the bookkeeping
// Table CB's topology reads. Vertex v of station j is index j·stride + v.
// Wall triangles come first, two per cell in (station, loop, segment) order;
// wallOf[k] is the profile vertex starting wall triangle k's segment.
type coilShell struct {
	verts         []r3.Vec
	vertexBound   []float64
	maxRound      float64
	delta         float64
	tris          [][3]int
	wallOf        []int
	walls         int
	capStartCount int
	reversed      bool
	stride        int
}

func (s coilShell) at(j int64, v int) int { return int(j)*s.stride + v }

// buildCoilShell is §5.2–§5.5: stations, held vertices, triangles, the one
// orientation decision, β and δ, and the crossing audit.
func buildCoilShell(ctx context.Context, rec coilRecord) (coilShell, error) {
	stride := len(rec.pts)
	n := rec.n
	sh := coilShell{stride: stride}

	// The per-vertex terms of the station formula, lifted once:
	// X(v, j) = P_v + (cos θ_j − 1)·B_v + t_j·D + sin θ_j·S_v.
	erU, erV := rec.axis.Radial()
	zero := coil.Point(new(big.Rat))
	pv := make([]proofbound.IvVec3, stride)
	bv := make([]proofbound.IvVec3, stride)
	sv := make([]proofbound.IvVec3, stride)
	tilt := big.NewRat(int64(rec.sigma*rec.axis.Side), 1)
	for v := range stride {
		pv[v] = rec.world.point(coil.Point(rec.u[v]), coil.Point(rec.v[v]), zero)
		bv[v] = rec.world.vector(proofbound.IntervalMul(rec.rho[v], erU), proofbound.IntervalMul(rec.rho[v], erV), zero)
		sv[v] = rec.world.vector(zero, zero, proofbound.IntervalScale(rec.rho[v], tilt))
	}
	pitchIv := coil.Point(rec.pitch)
	dvec := rec.world.vector(proofbound.IntervalMul(pitchIv, rec.axis.DU), proofbound.IntervalMul(pitchIv, rec.axis.DV), zero)
	one := coil.Point(big.NewRat(1, 1))

	total := int(n+1) * stride
	sh.verts = make([]r3.Vec, total)
	round := make([]float64, total)
	for j := int64(0); j <= n; j++ {
		if err := ctx.Err(); err != nil {
			return coilShell{}, err
		}
		t := coil.Fraction(rec.turns, j, n)
		sinT, cosT := coil.TurnSinCos(t)
		cm1 := proofbound.IntervalSub(cosT, one)
		for v := range stride {
			var coords [3]float64
			worst := 0.0
			for axis := range 3 {
				c := pv[v][axis]
				c = proofbound.IntervalAdd(c, proofbound.IntervalMul(cm1, bv[v][axis]))
				c = proofbound.IntervalAdd(c, proofbound.IntervalScale(dvec[axis], t))
				c = proofbound.IntervalAdd(c, proofbound.IntervalMul(sinT, sv[v][axis]))
				held, _ := coil.Mid(c).Float64()
				if math.IsNaN(held) || math.IsInf(held, 0) {
					return coilShell{}, fmt.Errorf(`%w: a coil station point runs past the representable float64 range`, ErrUnsupported)
				}
				coords[axis] = held
				worst = math.Max(worst, proofbound.IntervalFloatError(c, held))
			}
			k := sh.at(j, v)
			sh.verts[k] = r3.NewVec(coords[0], coords[1], coords[2])
			round[k] = proofbound.Radius3D(worst)
			sh.maxRound = math.Max(sh.maxRound, round[k])
		}
	}

	// Walls, ring by ring, in the prism's lateral order; then the caps from
	// one triangulation of the recorded region, capStart reversed and capEnd
	// retained (docs/loft-design.md §5's cap seeding).
	for j := range n {
		for _, idx := range rec.loopIdx {
			m := len(idx)
			for k := range m {
				v, w := idx[k], idx[(k+1)%m]
				b0, b1 := sh.at(j, v), sh.at(j, w)
				t0, t1 := sh.at(j+1, v), sh.at(j+1, w)
				sh.tris = append(sh.tris, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
				sh.wallOf = append(sh.wallOf, v, v)
			}
		}
	}
	sh.walls = len(sh.tris)
	capTris, err := triangulation.Triangulate(ctx, rec.pts, rec.loopIdx)
	if err != nil {
		return coilShell{}, wrapLoftTriangulationError(err)
	}
	for _, t := range capTris {
		sh.tris = append(sh.tris, [3]int{sh.at(0, t[0]), sh.at(0, t[2]), sh.at(0, t[1])})
	}
	sh.capStartCount = len(capTris)
	for _, t := range capTris {
		sh.tris = append(sh.tris, [3]int{sh.at(n, t[0]), sh.at(n, t[1]), sh.at(n, t[2])})
	}

	// One orientation decision for the whole shell: the exact signed
	// tetrahedron sum over the held floats.
	switch tessellation.OrientationSign(sh.verts, sh.tris, sh.verts[0]) {
	case 0:
		return coilShell{}, fmt.Errorf(`%w: the coil's held shell encloses no signed volume`, ErrUnsupported)
	case -1:
		sh.reversed = true
		for k, t := range sh.tris {
			sh.tris[k] = [3]int{t[0], t[2], t[1]}
		}
	}
	for k, t := range sh.tris {
		if loftmesh.TriangleCollapsed(sh.verts, t) {
			return coilShell{}, fmt.Errorf(`%w: rounding the coil's station points collapsed its triangle %d`, ErrUnsupported, k)
		}
	}

	// β (§5.4): every cell's departure — the helix sag at the matching
	// parameter, the cell's largest corner rounding (the bilinear patch
	// through the held corners against the one through the true corners), and
	// the held triangles' twist against that patch — charged to each of its
	// four corners.
	sh.vertexBound = append([]float64(nil), round...)
	dt := new(big.Rat).Quo(rec.turns, big.NewRat(n, 1))
	for j := range n {
		if err := ctx.Err(); err != nil {
			return coilShell{}, err
		}
		for _, idx := range rec.loopIdx {
			m := len(idx)
			for k := range m {
				v, w := idx[k], idx[(k+1)%m]
				rhoMax := rec.rho[v].Hi
				if rec.rho[w].Hi.Cmp(rhoMax) > 0 {
					rhoMax = rec.rho[w].Hi
				}
				sag := proofbound.RatFloatUp(coil.SagUpper(rhoMax, dt))
				corners := [4]int{sh.at(j, v), sh.at(j+1, v), sh.at(j, w), sh.at(j+1, w)}
				cellRound := 0.0
				for _, c := range corners {
					cellRound = math.Max(cellRound, round[c])
				}
				twist := proofbound.CellTwistOffsetUpper(sh.verts[corners[0]], sh.verts[corners[1]], sh.verts[corners[2]], sh.verts[corners[3]])
				departure := proofbound.AbsSumUpper(sag, cellRound, twist)
				if math.IsNaN(departure) || math.IsInf(departure, 0) {
					return coilShell{}, fmt.Errorf(`%w: a coil wall cell states no finite departure`, ErrUnsupported)
				}
				for _, c := range corners {
					sh.vertexBound[c] = math.Max(sh.vertexBound[c], departure)
				}
			}
		}
	}
	for _, b := range sh.vertexBound {
		sh.delta = math.Max(sh.delta, b)
	}

	if err := ctx.Err(); err != nil {
		return coilShell{}, err
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), sh.verts, sh.tris); err != nil {
		var contact *loftmesh.LoftContactError
		if errors.As(err, &contact) {
			// CS9: CP5 already proved the true solid simple, so a held
			// crossing is a station artefact, never a defect of the solid.
			return coilShell{}, fmt.Errorf(`%w: the coil's held triangles %d and %d %s; two turns pass closer than its %d stations per turn resolve`,
				ErrUnsupported, contact.I, contact.J, contact.Reason, coilStationsPerTurn)
		}
		return coilShell{}, err
	}
	return sh, nil
}

// coilHeld rounds an interval to its nearest float with the exact outward
// error, refusing a non-finite value (CS10).
func coilHeld(x coil.Iv, what string) (float64, float64, error) {
	held, _ := coil.Mid(x).Float64()
	if math.IsNaN(held) || math.IsInf(held, 0) {
		return 0, 0, fmt.Errorf(`%w: the coil's %s is not finite`, ErrUnsupported, what)
	}
	bound := proofbound.IntervalFloatError(x, held)
	if math.IsNaN(bound) || math.IsInf(bound, 0) {
		return 0, 0, fmt.Errorf(`%w: the coil's %s has no finite bound`, ErrUnsupported, what)
	}
	return held, bound, nil
}

// coilRatHeld is coilHeld over an exact rational.
func coilRatHeld(x *big.Rat, what string) (float64, float64, error) {
	held, _ := x.Float64()
	if math.IsNaN(held) || math.IsInf(held, 0) {
		return 0, 0, fmt.Errorf(`%w: the coil's %s is not finite`, ErrUnsupported, what)
	}
	return held, proofarith.RationalFloatError(x, held), nil
}
