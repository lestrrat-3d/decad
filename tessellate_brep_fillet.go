package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/filletband"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
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
	seamGap  []float64
	cells    []filletCell
	patches  int
}

// placeRings adds the band's interior ring vertices. It runs after place.
func (bc *brepBandChord) placeRings(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	if bc.partial != nil {
		return bc.placePartialRings(e, addVertex)
	}
	lm := &bc.lm
	match, cells, patches, err := tessellation.FilletLoopLayout(tessellation.FilletLoopInput{
		Walks: lm.walks, Joins: capBlendSampleJoins(lm.joins),
		Count: lm.count, ArcCount: lm.arcCount, SideStart: lm.sideStart,
		CapSamples: len(lm.capPts),
	})
	if err != nil {
		return err
	}
	n := bc.fillet.n
	side := make([]Point2, len(match))
	for c, index := range match {
		side[c] = lm.sidePts[index]
	}
	points, err := tessellation.FilletRingGeometry(tessellation.FilletRingInput{
		Side: side, Cap: lm.capPts, Radius: bc.band.setback.dc,
		RadiusDelta: bc.band.setback.dcDelta, CapLevel: bc.face.z0,
		CapLevelDelta: bc.face.z0Delta, SideLevel: bc.sideZ,
		MaterialSign: bc.band.matSign(bc.face), Count: n,
		Noun: "a fillet band", RingNoun: "a fillet",
	})
	if err != nil {
		return err
	}
	if len(bc.curved) > 0 {
		rIv, err := filletband.RadiusInterval(bc.band.setback.dc, bc.band.setback.dcDelta)
		if err != nil {
			return err
		}
		bc.fillet.seamGap, err = tessellation.CurvedMiterRings(points, tessellation.CurvedMiterInput{
			Walks: lm.walks, Curved: bc.curved, CapWallStart: lm.capWallStart,
			Radius: rIv, HeldRadius: bc.band.setback.dc,
			CapLevel: bc.face.z0, CapLevelDelta: bc.face.z0Delta,
			MaterialSign: bc.band.matSign(bc.face),
		})
		if err != nil {
			return err
		}
	}
	N := len(lm.capPts)
	fr := bc.fillet
	fr.match = match
	fr.cells, fr.patches = cells, patches
	fr.ringV = make([][]int, n+1)
	fr.dev = make([][]float64, n+1)
	fr.ringV[0] = make([]int, N)
	for c := range N {
		fr.ringV[0][c] = bc.sideV[match[c]]
	}
	fr.ringV[n] = bc.capV
	for k := 1; k < n; k++ {
		fr.ringV[k] = make([]int, N)
		fr.dev[k] = make([]float64, N)
		for c, point := range points[k] {
			fr.dev[k][c] = point.Delta
			fr.ringV[k][c] = addVertex(e.Canon(point.Point.U, point.Point.V, point.Z),
				proofbound.WalkEndBound{U: point.Delta, V: point.Delta})
		}
	}
	return nil
}

type filletCell = tessellation.FilletCell

// emitFillet writes the band's strips: for each cell and each of the n_φ
// strips between consecutive rings, the quad's two triangles, one where the
// side ring collapses to a reflex apex. Each patch states the distance its
// cells lie from its surface: the quarter-circle chord sagitta, the in-plane
// ring sagitta and the cells' twist (loop-fillet §7.1).
func (bc *brepBandChord) emitFillet(m *Mesh, faceOfRole func(string) (*Face, error), bump func(*Face, float64)) error {
	lm := &bc.lm
	fr := bc.fillet
	cells, patches := fr.cells, fr.patches
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
			a, b := fr.ringV[k][cell.C0], fr.ringV[k][cell.C1]
			A, B := fr.ringV[k+1][cell.C0], fr.ringV[k+1][cell.C1]
			if bc.start {
				tri(A, B, b, cell.Patch)
				tri(A, b, a, cell.Patch)
			} else {
				tri(a, b, B, cell.Patch)
				tri(a, B, A, cell.Patch)
			}
			if a != b {
				twist[cell.Patch] = math.Max(twist[cell.Patch],
					proofbound.CellTwistOffsetUpper(verts(k, cell.C0), verts(k, cell.C1), verts(k+1, cell.C0), verts(k+1, cell.C1)))
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
		eps := proofbound.AbsSumUpper(sPhi, ring, twist[p], fr.seamGap[i], fr.seamGap[(i+1)%n])
		bc.finishPatch(m, faces[p], eps, delta, levelDelta, axial, first[p], last[p], bump)
		p++
		if ni := (i + 1) % n; lm.joins != nil && lm.joins[ni].Arc {
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
	m.areaSlack = proofbound.AbsSumUpper(m.areaSlack, tessellation.FilletMeshAreaDeficit(m.vertices, m.triangles[first:last], face.area, face.areaBound))
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
