package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// CupRing is one chorded region loop, with the mesh vertices at each level
// and the source face of each outgoing wall chord.
type CupRing[F comparable] struct {
	Samples  []sectionrecord.Point2
	LoV, HiV []int
	Faces    []F
	Sag      float64
}

// CupAssembly contains facets in wall, kept-cap, pocket-floor, then rim order.
type CupAssembly[F comparable] struct {
	Triangles [][3]int
	Sources   []F
}

// AssembleCup shares each ring's vertices among its walls, floor and rim.
// The caller supplies the exact same clearance and triangulation error
// adapters used by the other tessellators.
func AssembleCup[F comparable](ctx context.Context, outer, cavity []CupRing[F], openIsMax bool,
	capStart, shellCap F, rimFace func(int) (F, error),
	clearance func(context.Context, []sectionrecord.Point2, [][]int, []float64) error,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error),
	faceTrim, faceAxial map[F]float64, outerDelta, cavityDelta, openDelta float64) (CupAssembly[F], error) {
	var mesh CupAssembly[F]
	addTriangle := func(tri [3]int, face F) {
		mesh.Triangles = append(mesh.Triangles, tri)
		mesh.Sources = append(mesh.Sources, face)
	}
	// A wall spans each outgoing chord's bottom and top vertices.
	addWalls := func(r CupRing[F]) {
		n := len(r.LoV)
		for j := range r.LoV {
			g0, g1 := j, (j+1)%n
			addTriangle([3]int{r.LoV[g0], r.LoV[g1], r.HiV[g1]}, r.Faces[j])
			addTriangle([3]int{r.LoV[g0], r.HiV[g1], r.HiV[g0]}, r.Faces[j])
		}
	}
	for i := range outer {
		addWalls(outer[i])
	}
	for i := range cavity {
		addWalls(cavity[i])
	}

	// Check every outer and cavity loop together before any cap triangulation.
	var clrPts []sectionrecord.Point2
	var clrIdx [][]int
	var clrSag []float64
	addClr := func(r CupRing[F]) {
		base := len(clrPts)
		clrPts = append(clrPts, r.Samples...)
		idx := make([]int, len(r.Samples))
		for k := range r.Samples {
			idx[k] = base + k
		}
		clrIdx = append(clrIdx, idx)
		clrSag = append(clrSag, r.Sag)
	}
	for i := range outer {
		addClr(outer[i])
	}
	for i := range cavity {
		addClr(cavity[i])
	}
	if err := clearance(ctx, clrPts, clrIdx, clrSag); err != nil {
		return CupAssembly[F]{}, err
	}

	openV := func(r CupRing[F]) []int {
		if openIsMax {
			return r.HiV
		}
		return r.LoV
	}
	floorV := func(r CupRing[F]) []int {
		if openIsMax {
			return r.LoV
		}
		return r.HiV
	}
	emit := func(tris [][3]int, vtx []int, reverse bool, face F) {
		for _, tri := range tris {
			a, b, c := vtx[tri[0]], vtx[tri[1]], vtx[tri[2]]
			if reverse {
				b, c = c, b
			}
			addTriangle([3]int{a, b, c}, face)
		}
	}

	// The outer region supplies the kept cap in its natural loop sense.
	oSamples := make([][]sectionrecord.Point2, len(outer))
	oFloor := make([][]int, len(outer))
	oReverse := make([]bool, len(outer))
	for i := range outer {
		oSamples[i] = outer[i].Samples
		oFloor[i] = floorV(outer[i])
	}
	capTris, capVtx, err := cupTriangulate(ctx, oSamples, oFloor, oReverse, triangulate)
	if err != nil {
		return CupAssembly[F]{}, err
	}
	emit(capTris, capVtx, openIsMax, capStart)
	for i := range outer {
		faceTrim[capStart] = math.Max(faceTrim[capStart], outer[i].Sag)
	}
	faceAxial[capStart] = outerDelta

	// The reversed cavity loops are flipped back for the pocket floor.
	cSamples := make([][]sectionrecord.Point2, len(cavity))
	cCav := make([][]int, len(cavity))
	cReverse := make([]bool, len(cavity))
	for i := range cavity {
		cSamples[i] = cavity[i].Samples
		cCav[i] = floorV(cavity[i])
		cReverse[i] = true
	}
	shellTris, shellVtx, err := cupTriangulate(ctx, cSamples, cCav, cReverse, triangulate)
	if err != nil {
		return CupAssembly[F]{}, err
	}
	emit(shellTris, shellVtx, !openIsMax, shellCap)
	for i := range cavity {
		faceTrim[shellCap] = math.Max(faceTrim[shellCap], cavity[i].Sag)
	}
	faceAxial[shellCap] = cavityDelta

	// Rim zero spans both outer loops; each post rim swaps the two roles.
	for i := range outer {
		rim, err := rimFace(i)
		if err != nil {
			return CupAssembly[F]{}, err
		}
		var samples [][]sectionrecord.Point2
		var vtxRings [][]int
		if i == 0 {
			samples = [][]sectionrecord.Point2{outer[0].Samples, cavity[0].Samples}
			vtxRings = [][]int{openV(outer[0]), openV(cavity[0])}
		} else {
			samples = [][]sectionrecord.Point2{cavity[i].Samples, outer[i].Samples}
			vtxRings = [][]int{openV(cavity[i]), openV(outer[i])}
		}
		rimTris, rimVtx, err := cupTriangulate(ctx, samples, vtxRings, []bool{false, false}, triangulate)
		if err != nil {
			return CupAssembly[F]{}, err
		}
		emit(rimTris, rimVtx, !openIsMax, rim)
		faceTrim[rim] = math.Max(faceTrim[rim], math.Max(outer[i].Sag, cavity[i].Sag))
		faceAxial[rim] = openDelta
	}
	return mesh, nil
}

// cupTriangulate triangulates a cup patch over already allocated ring vertices.
// reverse restores the polygon-with-holes loop sense required by the cap
// triangulator; the returned indices still name the original ring vertices.
func cupTriangulate(ctx context.Context, samples [][]sectionrecord.Point2, vtxRings [][]int, reverse []bool,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error)) ([][3]int, []int, error) {
	var pts []sectionrecord.Point2
	var vtx []int
	var loops [][]int
	for i := range samples {
		base := len(pts)
		pts = append(pts, samples[i]...)
		vtx = append(vtx, vtxRings[i]...)
		idx := make([]int, len(samples[i]))
		for k := range samples[i] {
			idx[k] = base + k
		}
		if reverse[i] {
			for a, z := 0, len(idx)-1; a < z; a, z = a+1, z-1 {
				idx[a], idx[z] = idx[z], idx[a]
			}
		}
		loops = append(loops, idx)
	}
	tris, err := triangulate(ctx, pts, loops)
	return tris, vtx, err
}
