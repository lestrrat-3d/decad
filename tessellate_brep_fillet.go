package decad

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the brep tessellator's reading of a fillet band
// (docs/loop-fillet-design.md §7.1, Table DF's DF4 and DF5). The band is
// chorded as a chamfer band is (tessellate_brep_band.go): ONE count per wall
// walk, shared by the side ring on the trimmed wall and the cap contour ring
// on F. Between those two rings the band holds n_φ − 1 interior rings. Ring k
// sits at φ_k = k·(π/2)/n_φ, k = 0 the side ring and k = n_φ the cap ring, at
// height h_k = r·sin φ_k from the side level toward F, and its vertices are
// the affine interpolation of the matched side and cap vertices at fraction
// t_k/r = 1 − cos φ_k, which lies on the pipe for every admitted piece. At a
// reflex corner every connector sample matches the corner's one apex vertex,
// so the horn torus's first strip is a fan.

// filletRings is one fillet band's interior rings beside the ring tables the
// emitter reads.
type filletRings struct {
	// n is n_φ, the strip count between the side ring and the cap ring.
	n int
	// match is, per cap sample, the side sample it pairs with.
	match []int
	// ringV[k][c] is ring k's mesh vertex at cap sample c; ring 0 is the side
	// ring's vertex and ring n the cap ring's.
	ringV [][]int
	// dev[k][c] is interior ring vertex (k, c)'s displacement from the exact
	// point of the denoted pipe at φ_k, beside its two ends' own.
	dev [][]float64
	// patchEps is each patch face's deviation of its mesh from its surface.
	patchEps map[*Face]float64
}

// filletRingCount is n_φ: the fewest strips whose quarter-circle chord
// sagitta r·(1 − cos(Δφ/2)) stays within half the chord budget, at least 2.
func filletRingCount(r, chord float64) int {
	tol := chord / 2
	if !(r > 0) || !(tol > 0) {
		return 2
	}
	arg := 1 - tol/r
	if arg < -1 {
		arg = -1
	}
	n := int(math.Ceil((math.Pi / 2) / (2 * math.Acos(arg))))
	return min(max(n, 2), 4096)
}

// filletMatch pairs each cap sample with its side sample, in the order
// tessellation.SampleCapBlend emits the cap samples.
func (bc *brepBandChord) filletMatch() ([]int, error) {
	lm := &bc.lm
	n := len(lm.walks)
	var match []int
	for i, w := range lm.walks {
		nw := 1
		if w.IsCircular() {
			nw = lm.count[i]
		}
		for k := range nw {
			match = append(match, lm.sideStart[i]+k)
		}
		ni := (i + 1) % n
		if lm.joins != nil && lm.joins[ni].arc {
			for range lm.arcCount[ni] {
				match = append(match, lm.sideStart[ni])
			}
		}
	}
	if len(match) != len(lm.capPts) {
		return nil, fmt.Errorf(`%w: fillet band's cap ring holds %d samples for %d matched side samples`, ErrUnsupported, len(lm.capPts), len(match))
	}
	return match, nil
}

// placeRings adds the band's interior ring vertices. It runs after place.
func (bc *brepBandChord) placeRings(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	match, err := bc.filletMatch()
	if err != nil {
		return err
	}
	lm := &bc.lm
	n := bc.fillet.n
	r, rDelta := bc.band.setback.dc, bc.band.setback.dcDelta
	m := bc.band.matSign(bc.face)
	z0, z0Delta := bc.face.z0, bc.face.z0Delta
	rz0, rzd, rr, rrd := proofarith.FloatRat(z0), proofarith.FloatRat(z0Delta), proofarith.FloatRat(r), proofarith.FloatRat(rDelta)
	if rz0 == nil || rzd == nil || rr == nil || rrd == nil {
		return fmt.Errorf(`%w: a fillet band's radius or level is not finite`, ErrNotFinite)
	}
	rIv := proofbound.IntervalWiden(proofbound.PointInterval(rr), rrd)
	one := proofbound.PointInterval(big.NewRat(1, 1))
	mRat := big.NewRat(int64(m), 1)

	N := len(lm.capPts)
	fr := bc.fillet
	fr.match = match
	fr.ringV = make([][]int, n+1)
	fr.dev = make([][]float64, n+1)
	fr.ringV[0] = make([]int, N)
	for c := range N {
		fr.ringV[0][c] = bc.sideV[match[c]]
	}
	fr.ringV[n] = bc.capV
	for k := 1; k < n; k++ {
		phi := float64(k) * (math.Pi / 2) / float64(n)
		rphi := proofarith.FloatRat(phi)
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(rphi)
		if !ok {
			return fmt.Errorf(`%w: a fillet ring's angle has no sine enclosure`, ErrUnsupported)
		}
		f := 1 - math.Cos(phi)
		fRat := proofarith.FloatRat(f)
		errF := proofbound.IntervalFloatError(proofbound.IntervalSub(one, cosIv), f)
		h := r * math.Sin(phi)
		z := bc.sideZ - m*h
		zIv := proofbound.IntervalWiden(proofbound.IntervalAdd(proofbound.PointInterval(rz0),
			proofbound.IntervalScale(proofbound.IntervalMul(rIv, proofbound.IntervalSub(one, sinIv)), mRat)), rzd)
		errZ := proofbound.IntervalFloatError(zIv, z)
		if fRat == nil || proofbound.IsNonFinite(errF) || proofbound.IsNonFinite(errZ) {
			return fmt.Errorf(`%w: a fillet ring's position is not finite`, ErrNotFinite)
		}
		fr.ringV[k] = make([]int, N)
		fr.dev[k] = make([]float64, N)
		for c := range N {
			s, cp := lm.sidePts[match[c]], lm.capPts[c]
			u, ru, okU := interpolate(s.U, cp.U, f, fRat)
			v, rv, okV := interpolate(s.V, cp.V, f, fRat)
			if !okU || !okV {
				return fmt.Errorf(`%w: a fillet ring's position is not finite`, ErrNotFinite)
			}
			span := proofbound.AbsSumUpper(math.Abs(cp.U-s.U), math.Abs(cp.V-s.V))
			dev := proofbound.AbsSumUpper(ru, rv, proofbound.ProductUpper(errF, span), errZ)
			fr.dev[k][c] = dev
			fr.ringV[k][c] = addVertex(e.Canon(u, v, z), proofbound.WalkEndBound{U: dev, V: dev})
		}
	}
	return nil
}

// interpolate is s + f·(c − s) in float64 beside the exact rounding error of
// that evaluation against the rational value of the same formula. The error
// is not usable where ok is false (an input is not finite).
func interpolate(s, c, f float64, fRat *big.Rat) (float64, float64, bool) {
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

// filletCell is one strip cell: cap samples c0 and c1 of patch p.
type filletCell struct{ c0, c1, patch int }

// filletCells lists the band's cells in patch order: walk i's patch, then the
// patch of the reflex corner after it.
func (bc *brepBandChord) filletCells() ([]filletCell, int, error) {
	lm := &bc.lm
	n := len(lm.walks)
	N := len(lm.capPts)
	var cells []filletCell
	c, p := 0, 0
	for i, w := range lm.walks {
		nw := 1
		if w.IsCircular() {
			nw = lm.count[i]
		}
		for range nw {
			cells = append(cells, filletCell{c, (c + 1) % N, p})
			c++
		}
		p++
		ni := (i + 1) % n
		if lm.joins != nil && lm.joins[ni].arc {
			for range lm.arcCount[ni] {
				cells = append(cells, filletCell{c, (c + 1) % N, p})
				c++
			}
			p++
		}
	}
	if c != N {
		return nil, 0, fmt.Errorf(`%w: fillet band cells cover %d of %d cap samples`, ErrUnsupported, c, N)
	}
	return cells, p, nil
}

// emitFillet writes the band's strips: for each cell and each of the n_φ
// strips between consecutive rings, the quad's two triangles, one where the
// side ring collapses to a reflex apex. Each patch states the distance its
// cells lie from its surface: the quarter-circle chord sagitta, the in-plane
// ring sagitta and the cells' twist (loop-fillet §7.1).
func (bc *brepBandChord) emitFillet(m *Mesh, faceOfRole func(string) (*Face, error), bump func(*Face, float64)) error {
	lm := &bc.lm
	fr := bc.fillet
	cells, patches, err := bc.filletCells()
	if err != nil {
		return err
	}
	cbp := bc.cbp
	matSign, capZ := 1.0, cbp.z0
	if !bc.start {
		matSign, capZ = -1, cbp.z1
	}
	setback := cbp.setbackAt(matSign)
	delta := cbp.bandDelta[capBandKey{loop: 0, start: bc.start}]
	levelDelta := proofbound.AbsSumUpper(setback.dsDelta, proofarith.AddRoundError(capZ, matSign*setback.ds, bc.sideZ))
	axial := cbp.capBandLevel(capZ, matSign).Bound

	r := setback.axialUpper()
	dphi := proofbound.UpRound(proofbound.UpRound(math.Pi/2*(1+1e-12)) / float64(fr.n))
	sPhi := proofbound.ProductUpper(r, proofbound.UpRound(proofbound.ProductUpper(dphi, dphi)/8))

	faces := make([]*Face, patches)
	twist := make([]float64, patches)
	first := make([]int, patches)
	last := make([]int, patches)
	for p := range patches {
		face, err := faceOfRole(bc.band.patchRole(p))
		if err != nil {
			return err
		}
		faces[p] = face
		first[p] = -1
	}
	tri := func(a, b, c, p int) {
		if a == b || b == c || a == c {
			return
		}
		if first[p] < 0 {
			first[p] = len(m.triangles)
		}
		m.addTriangle([3]int{a, b, c}, faces[p])
		last[p] = len(m.triangles)
	}
	verts := func(k, c int) r3.Vec { return m.vertices[fr.ringV[k][c]] }
	for _, cell := range cells {
		for k := range fr.n {
			a, b := fr.ringV[k][cell.c0], fr.ringV[k][cell.c1]
			A, B := fr.ringV[k+1][cell.c0], fr.ringV[k+1][cell.c1]
			if bc.start {
				tri(A, B, b, cell.patch)
				tri(A, b, a, cell.patch)
			} else {
				tri(a, b, B, cell.patch)
				tri(a, B, A, cell.patch)
			}
			if a != b {
				twist[cell.patch] = math.Max(twist[cell.patch],
					proofbound.CellTwistOffsetUpper(verts(k, cell.c0), verts(k, cell.c1), verts(k+1, cell.c0), verts(k+1, cell.c1)))
			}
		}
	}

	fr.patchEps = map[*Face]float64{}
	n := len(lm.walks)
	p := 0
	for i := range n {
		ring := 0.0
		if lm.walks[i].IsCircular() {
			ring = math.Max(lm.sideSag[i], lm.capSag[i])
		}
		eps := proofbound.AbsSumUpper(sPhi, ring, twist[p])
		bc.finishPatch(m, faces[p], eps, delta, levelDelta, axial, first[p], last[p], bump)
		p++
		if ni := (i + 1) % n; lm.joins != nil && lm.joins[ni].arc {
			eps := proofbound.AbsSumUpper(sPhi, lm.arcSag[ni], twist[p])
			bc.finishPatch(m, faces[p], eps, delta, levelDelta, axial, first[p], last[p], bump)
			p++
		}
	}
	return nil
}

// finishPatch publishes one patch's face displacement and its area slack.
func (bc *brepBandChord) finishPatch(m *Mesh, face *Face, eps, delta, levelDelta, axial float64, first, last int, bump func(*Face, float64)) {
	bc.fillet.patchEps[face] = eps
	bump(face, proofbound.AbsSumUpper(eps, delta, levelDelta, axial))
	if first < 0 {
		return
	}
	m.areaSlack = proofbound.AbsSumUpper(m.areaSlack, meshAreaDeficit(m.vertices, m.triangles[first:last], face.area, face.areaBound))
}

// meshAreaDeficit bounds how far the held facets' total area lies from a
// surface area trueArea ± trueBound: the facets' area is enclosed in exact
// rational arithmetic over the held vertices and compared at the far ends.
// A facet the enclosure cannot state answers +Inf.
func meshAreaDeficit(verts []r3.Vec, tris [][3]int, trueArea, trueBound float64) float64 {
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

// arcMotion fills in the motion of a reflex connector's cap samples, which the
// chamfer's motion proof leaves unbounded (capBlendCapMotion). A fillet band's
// connector sample lies on the circle of radius r about the corner at the
// station its azimuth names, within its station bound, plus the radius's unit
// conversion and the contour displacement the connector's ends carry
// (loop-fillet DF5: the fan's stations are the connector arc's azimuths at
// every ring).
func (fr *filletRings) arcMotion(lm *capBlendLoopMesh, faceDelta, radiusDelta float64) {
	for i, start := range lm.capArcStart {
		if start < 0 {
			continue
		}
		for j := start; j < start+lm.arcCount[i]; j++ {
			lm.capMotion[j] = proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(lm.capBound[j]), radiusDelta, faceDelta)
		}
	}
}
