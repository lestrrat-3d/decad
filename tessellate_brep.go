package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/units"
)

// brepWallMesh is one swept face's chording: its rim polylines at z0 and z1 as
// mesh vertex indices in walk order (closed for a whole circle), its largest
// sagitta, and the chord-versus-arc charges tessellation §5 reads.
type brepWallMesh struct {
	// cols holds one vertex column per rim sample, ascending from z0 to z1,
	// and levels each column's heights: a side line's column stops at its
	// own splits alone, so the neighbouring face across it meets exactly the
	// side pieces' vertices, and every interior column at every split of
	// either side. bottom and top are the rims, each column's first and last
	// vertex.
	cols        [][]int
	levels      [][]float64
	bottom, top []int
	sag         float64
	wallSlack   float64
	capSlack    float64
	segmentArea float64
}

// tessellateBrep meshes a brepPayload (docs/general-boolean-design.md §4.4).
// Every swept face chords its own wall once with tessellation.SampleLoop, the
// sampler a prism's walls use, and emits its rim polylines at both levels.
// Every planar face triangulates its loops with triangulation.Triangulate, as a
// prism cap does, over the polylines of the edges it shares, so every edge is
// chorded from one sample set and the mesh closes by construction;
// RequireClosedMesh proves it. A mesh vertex is keyed by its reference
// coordinates (brepEmbeds), which a signed permutation states exactly.
func tessellateBrep(ctx context.Context, b *Body, bp brepPayload, chord float64, verify Verification) (*Mesh, error) {
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	// The largest section displacement is reserved from the chord budget, as
	// tessellatePrism reserves its own.
	section := bp.sectionDelta()
	budget := chord
	if section > 0 {
		budget = freeform.DownRound(freeform.DownRound(chord - section))
		if budget <= 0 {
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s`,
				ErrUnsupported, units.Millimeters(chord), units.Millimeters(section))
		}
	}
	byRole := map[string]*Face{}
	for _, face := range b.Faces() {
		for _, origin := range face.Origins() {
			byRole[origin.Role] = face
		}
	}
	faceOf := func(fi int) (*Face, error) {
		face := byRole[bp.faces[fi].role]
		if face == nil {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, bp.faces[fi].role)
		}
		return face, nil
	}

	bands, imposed, err := brepChordBands(ctx, bp, topo, budget)
	if err != nil {
		return nil, err
	}
	faceOfRole := func(role string) (*Face, error) {
		face := byRole[role]
		if face == nil {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return face, nil
	}

	refView := bp.refView()
	var mesh Mesh
	var store, round []float64
	var canon [][3]float64
	index := map[[3]float64]int{}
	addVertex := func(c [3]float64, sample proofbound.WalkEndBound) int {
		allow := proofbound.WalkEndBoundAllow(sample)
		if i, ok := index[c]; ok {
			store[i] = math.Max(store[i], proofbound.AbsSumUpper(allow, round[i]))
			return i
		}
		v := refView.point(c[0], c[1], c[2])
		r := exactPrismPointRound(refView, c[0], c[1], c[2], v)
		mesh.vertices = append(mesh.vertices, v)
		round = append(round, r)
		store = append(store, proofbound.AbsSumUpper(allow, r))
		canon = append(canon, c)
		index[c] = len(canon) - 1
		return len(canon) - 1
	}
	faceTrim := map[*Face]float64{}
	faceAxial := map[*Face]float64{}
	edgePoly := make([][]int, len(topo.edges))
	walls := map[int]brepWallMesh{}
	work := freeform.NewFreeformWork()
	for fi, f := range bp.faces {
		if f.planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		var samples *tessellation.ChordSamples[*Face]
		if s, ok := imposed[fi]; ok {
			samples = &s
		}
		wm, err := brepChordWall(ctx, f, topo.walls[fi], topo.embeds[fi], face, budget, work, samples, addVertex)
		if err != nil {
			return nil, err
		}
		walls[fi] = wm
		for _, ui := range topo.faceUses[fi] {
			// A rim on a band's side contour is open: the band, not another
			// face, meets it, and no edge holds its polyline.
			switch u := topo.uses[ui]; u.Part {
			case brepRim0:
				if ei := topo.edgeOf[ui]; ei >= 0 {
					edgePoly[ei] = wm.bottom
				}
			case brepRim1:
				if ei := topo.edgeOf[ui]; ei >= 0 {
					edgePoly[ei] = wm.top
				}
			case brepSide0, brepSide1:
				// The piece's two vertices are the wall's own row ends at
				// its levels, keyed by the same reference coordinates.
				edgePoly[topo.edgeOf[ui]] = []int{addVertex(u.DirFrom, proofbound.WalkEndBound{}), addVertex(u.DirTo, proofbound.WalkEndBound{})}
			}
		}
		for j := 0; j+1 < len(wm.cols); j++ {
			brepZipColumns(wm.cols[j], wm.cols[j+1], wm.levels[j], wm.levels[j+1], func(tri [3]int) {
				mesh.addTriangle(tri, face)
			})
		}
		faceTrim[face] = wm.sag
		faceAxial[face] = proofbound.AbsSumUpper(math.Max(f.z0Delta, f.z1Delta), f.delta)
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, wm.wallSlack)
	}
	if err := brepWallClearance(ctx, bp, topo, walls, canon); err != nil {
		return nil, err
	}

	// A band's rings are placed once every wall has placed its samples: a wall
	// beside a band took the band's own side-ring points, so the two name the
	// same vertices.
	bandsOf := map[int][]int{}
	for bi := range bands {
		bands[bi].place(topo.embeds[bands[bi].band.face], addVertex)
		if bands[bi].fillet != nil {
			if err := bands[bi].placeRings(topo.embeds[bands[bi].band.face], addVertex); err != nil {
				return nil, err
			}
		}
		bandsOf[bands[bi].band.face] = append(bandsOf[bands[bi].band.face], bi)
	}

	// faceCapSlack is each planar face's own share of its curved edges'
	// chord-versus-arc area: one cap's share per curved edge it bounds.
	faceCapSlack := map[*Face]float64{}
	for fi, f := range bp.faces {
		if !f.planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		e := topo.embeds[fi]
		loops := make([][]int, len(topo.planar[fi]))
		loopSag := make([]float64, len(loops))
		var pts []Point2
		var meshIdx []int
		trim, capSlack := 0.0, 0.0
		ringLoop := map[int]int{}
		for _, bi := range bandsOf[fi] {
			ringLoop[bands[bi].band.loop] = bi
		}
		for _, ui := range topo.faceUses[fi] {
			u := topo.uses[ui]
			ei := topo.edgeOf[ui]
			var poly []int
			switch {
			case ei >= 0:
				owner := topo.uses[topo.edges[ei][0]]
				poly = edgePoly[ei]
				if poly == nil {
					poly = []int{addVertex(owner.DirFrom, owner.StartBound()), addVertex(owner.DirTo, owner.EndBound())}
					edgePoly[ei] = poly
				}
				if !topo.forward(ui) {
					poly = slices.Clone(poly)
					slices.Reverse(poly)
				}
				if wm, ok := walls[owner.Face]; ok && brepIsRim(owner.Part) {
					trim = math.Max(trim, wm.sag)
					loopSag[u.Loop] = math.Max(loopSag[u.Loop], wm.sag)
					capSlack = proofbound.AbsSumUpper(capSlack, wm.capSlack)
				}
			default:
				// An open use is a band's boundary. The band's cap contour
				// on this face is its whole ring, placed below; a segment of
				// a (pl) face on the band's side contour is a straight line
				// between its two held ends.
				if _, ok := ringLoop[u.Loop]; ok {
					continue
				}
				poly = []int{addVertex(u.DirFrom, u.StartBound()), addVertex(u.DirTo, u.EndBound())}
			}
			for _, vi := range poly[:len(poly)-1] {
				local := e.Local(canon[vi])
				pts = append(pts, Point2{U: local[0], V: local[1]})
				meshIdx = append(meshIdx, vi)
				loops[u.Loop] = append(loops[u.Loop], len(pts)-1)
			}
		}
		for _, bi := range bandsOf[fi] {
			bc := &bands[bi]
			proof := bc.lm.proof()
			sag := tessellation.CapBlendRingSagitta(proof, true)
			for j, p := range bc.lm.capPts {
				pts = append(pts, p)
				meshIdx = append(meshIdx, bc.capV[j])
				loops[bc.band.loop] = append(loops[bc.band.loop], len(pts)-1)
			}
			trim = math.Max(trim, sag)
			loopSag[bc.band.loop] = math.Max(loopSag[bc.band.loop], sag)
			capSlack = proofbound.AbsSumUpper(capSlack, tessellation.CapBlendRingSegmentArea(proof, true, bc.band.setback.dc))
		}
		if err := requireLoopClearance(ctx, pts, loops, loopSag); err != nil {
			return nil, err
		}
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		if err != nil {
			return nil, err
		}
		for _, tri := range tris {
			a, bb, c := meshIdx[tri[0]], meshIdx[tri[1]], meshIdx[tri[2]]
			if !f.outward {
				bb, c = c, bb
			}
			mesh.addTriangle([3]int{a, bb, c}, face)
		}
		faceTrim[face] = trim
		faceAxial[face] = proofbound.AbsSumUpper(f.z0Delta, f.delta)
		faceCapSlack[face] = capSlack
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, capSlack)
	}
	if len(bands) > 0 {
		geomOf := func(bi int) map[string]capPatchGeom {
			out := map[string]capPatchGeom{}
			if bi < len(bp.loopPatches) {
				for _, p := range bp.loopPatches[bi] {
					out[p.role] = p.geom
				}
			}
			return out
		}
		bump := func(f *Face, v float64) { faceTrim[f] = math.Max(faceTrim[f], v) }
		wb := proofbound.NewWorkBudget(ctx)
		for bi := range bands {
			if err := bands[bi].emit(wb, &mesh, geomOf(bi), faceOfRole, bump); err != nil {
				return nil, err
			}
		}
	}
	if bp.xform.IsReflection() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	if err := liftTessellationError(tessellation.RequireClosedMesh(mesh.triangles)); err != nil {
		return nil, err
	}
	if len(bands) > 0 {
		// A band's strips and fans can collapse where its rings meet; the cap
		// blend tessellator refuses the same two faults (tessellate_capblend.go).
		if err := tessellation.RequireVertexLinks(ctx, len(mesh.vertices), mesh.triangles); err != nil {
			return nil, err
		}
		if err := requireCapBlendFacetAreas(&mesh, "chamfered brep"); err != nil {
			return nil, err
		}
	}
	storeMax, err := requireDerivableStore(store)
	if err != nil {
		return nil, err
	}
	if err := composeFaceBounds(&mesh, faceTrim, faceAxial, store, 0); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return &mesh, nil
	}

	// Occupied volume (tessellation §5's prism term applied per face): each
	// curved wall's circular-segment slivers over its height, each wall's
	// section band over its height, each planar face's level displacement over
	// its area, and every computed coordinate's swept volume. Absolute sums
	// throughout: an occupied-volume bound admits no cancellation.
	areaUpper := meshFaceAreaUpper(&mesh, store)
	var terms []float64
	for fi, f := range bp.faces {
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		if f.planar() {
			region := brepPlanarDisplacement(f, topo.planar[fi])
			mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, region)
			terms = append(terms, proofbound.ProductUpper(f.z0Delta,
				proofbound.AbsSumUpper(areaUpper[face], faceCapSlack[face], region)))
			continue
		}
		w := topo.walls[fi]
		height := proofbound.UpRound(f.z1 - f.z0)
		moved := proofbound.AbsSumUpper(height, f.z0Delta, f.z1Delta)
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
			proofbound.ProductUpper(proofbound.SectionDisplacementLength(f.delta, 1), moved))
		terms = append(terms,
			proofbound.ProductUpper(height, walls[fi].segmentArea),
			proofbound.ProductUpper(moved, proofbound.SectionDisplacementArea(f.delta, 1, proofbound.AbsSumUpper(w.Length, w.LengthBound))))
	}
	mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, store))
	if len(bands) > 0 {
		// A band the cap-loop chamfer's occupied-volume proof does not admit
		// leaves the mesh export-only (docs/tessellation-reach-design.md §7),
		// and the mesh boolean refuses it with the same reason.
		refusal, err := brepBandsOccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), bp)
		if err != nil {
			return nil, err
		}
		if refusal != nil {
			mesh.symDiffOK = false
			return &mesh, nil //nolint:nilerr // refusal is the admission result; err was checked above
		}
		motion, err := brepBandMotion(ctx, bands, store, round)
		if err != nil {
			return nil, err
		}
		if storeMax, err = requireDerivableStore(motion); err != nil {
			return nil, err
		}
		terms = append(terms, brepBandChordVolume(bands))
		for bi := range bands {
			// A fillet strip lies within its patch's stated distance of the
			// pipe it chords: the symmetric difference is at most its area
			// times that distance (loop-fillet §7.1).
			if bands[bi].fillet == nil {
				continue
			}
			for face, eps := range bands[bi].fillet.patchEps {
				terms = append(terms, proofbound.ProductUpper(areaUpper[face], eps))
			}
		}
	}
	terms = append(terms, proofbound.SweptVolumeAllow(storeMax,
		proofbound.PerturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax)))
	if err := publishSymDiff(&mesh, terms); err != nil {
		return nil, err
	}
	return &mesh, nil
}

// brepPlanarDisplacement is the area a planar face's section displacement
// moves: proofbound.SectionDisplacementArea over its own walks.
func brepPlanarDisplacement(f brepFace, walks [][]survey2d.SegmentWalk) float64 {
	if f.delta == 0 {
		return 0
	}
	count := 0
	perimeter := 0.0
	for _, loop := range walks {
		for _, w := range loop {
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
			count++
		}
	}
	return proofbound.SectionDisplacementArea(f.delta, count, perimeter)
}

// brepChordWall chords one swept face's wall with tessellation.SampleLoop
// over its one walk and places its rim samples at both levels. An open wall's
// polyline ends at its walk's end; a whole circle's closes on its first
// sample. Each end sample is charged the bound from its held point to the
// point the wall's record denotes there (boundarywalk.DenotedStartBound and
// DenotedEndBound), which adds an arc's radial residual at its natural t = 1
// end; addVertex keeps the largest bound any use places at one vertex.
// imposed, when non-nil, replaces the chording with the samples a route L band
// states for this wall (brepImposeWall); the wall's own end is still appended.
func brepChordWall(ctx context.Context, f brepFace, w survey2d.SegmentWalk, e brepEmbed, face *Face, chord float64,
	work *freeform.FreeformWork, imposed *tessellation.ChordSamples[*Face], addVertex func([3]float64, proofbound.WalkEndBound) int) (brepWallMesh, error) {
	walk := survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}
	var sampled tessellation.ChordSamples[*Face]
	if imposed != nil {
		sampled = *imposed
	} else {
		var err error
		sampled, err = tessellation.SampleLoop[*Face]([]survey2d.SideWalk{walk}, []CurveSegment{f.wall}, chord,
			f.z1-f.z0, work, proofbound.NewWorkBudget(ctx), func(survey2d.SideWalk) (*Face, error) { return face, nil },
			stationbound.ChordStationBound)
		if err != nil {
			return brepWallMesh{}, err
		}
	}
	samples, bounds := sampled.Samples, sampled.BoundOf
	if !w.Closed {
		samples = append(samples, Point2{U: w.EndU, V: w.EndV})
		bounds = append(bounds, boundarywalk.DenotedEndBound(f.wall, w))
	}
	wm := brepWallMesh{sag: sampled.MaxSag, wallSlack: sampled.WallSlack, capSlack: sampled.CapSlack,
		segmentArea: sampled.SegmentArea}
	// Each side line's column holds a vertex at its own splits alone, so the
	// side pieces' vertices are the wall's own and the face across the side
	// line meets no vertex it does not hold; the interior columns hold every
	// split of either side, and brepZipColumns closes the strips between.
	levelsOf := func(splits ...[]brepSplit) []float64 {
		out := []float64{f.z0}
		for _, side := range splits {
			for _, sp := range side {
				out = append(out, sp.Z)
			}
		}
		out = append(out, f.z1)
		slices.Sort(out)
		return slices.Compact(out)
	}
	every := levelsOf(f.side0, f.side1)
	for j, p := range samples {
		levels := every
		switch {
		case w.Closed:
		case j == 0:
			levels = levelsOf(f.side0)
		case j == len(samples)-1:
			levels = levelsOf(f.side1)
		}
		col := make([]int, 0, len(levels))
		for _, z := range levels {
			col = append(col, addVertex(e.Canon(p.U, p.V, z), bounds[j]))
		}
		wm.cols = append(wm.cols, col)
		wm.levels = append(wm.levels, levels)
		wm.bottom = append(wm.bottom, col[0])
		wm.top = append(wm.top, col[len(col)-1])
	}
	if w.Closed {
		wm.cols = append(wm.cols, wm.cols[0])
		wm.levels = append(wm.levels, wm.levels[0])
		wm.bottom = append(wm.bottom, wm.bottom[0])
		wm.top = append(wm.top, wm.top[0])
	}
	return wm, nil
}

// brepZipColumns triangulates the strip between two adjacent vertex columns
// of one wall, a and b ascending at heights za and zb from one common bottom
// to one common top: it climbs whichever column's next vertex is lower, b
// first at a shared height, so two columns holding the same heights close
// in quads split as (a, b, b↑) and (a, b↑, a↑).
func brepZipColumns(a, b []int, za, zb []float64, emit func([3]int)) {
	i, k := 0, 0
	for i < len(a)-1 || k < len(b)-1 {
		if i == len(a)-1 || (k < len(b)-1 && zb[k+1] <= za[i+1]) {
			emit([3]int{a[i], b[k], b[k+1]})
			k++
			continue
		}
		emit([3]int{a[i], b[k], a[i+1]})
		i++
	}
}

// brepWallClearance refuses a mesh whose chorded walls could cross where the
// analytic walls do not: every two walls sweeping along one reference axis over
// overlapping intervals, and sharing no endpoint, must clear each other by
// more than their summed sagittae plus the section's rounding floor — the
// per-slab loop clearance the stacked tessellator proves, read per wall pair.
// Each open wall polyline is walked out and back so the loop clearance reader
// measures exactly its own chords.
func brepWallClearance(ctx context.Context, bp brepPayload, topo *brepTopology, walls map[int]brepWallMesh, canon [][3]float64) error {
	type wallLine struct {
		axis   int
		lo, hi float64
		pts    []Point2
		sag    float64
	}
	var lines []wallLine
	for fi, f := range bp.faces {
		wm, ok := walls[fi]
		if !ok {
			continue
		}
		e := topo.embeds[fi]
		k := e.Axis[2]
		lo, hi := e.Sign[2]*f.z0, e.Sign[2]*f.z1
		if lo > hi {
			lo, hi = hi, lo
		}
		line := wallLine{axis: k, lo: lo, hi: hi, sag: wm.sag}
		for _, vi := range wm.bottom {
			c := canon[vi]
			var p [2]float64
			n := 0
			for i, x := range c {
				if i != k {
					p[n] = x
					n++
				}
			}
			line.pts = append(line.pts, Point2{U: p[0], V: p[1]})
		}
		lines = append(lines, line)
	}
	for i := range lines {
		for j := i + 1; j < len(lines); j++ {
			if err := ctx.Err(); err != nil {
				return err
			}
			a, b := lines[i], lines[j]
			if a.axis != b.axis || math.Max(a.lo, b.lo) >= math.Min(a.hi, b.hi) {
				continue
			}
			ends := []Point2{a.pts[0], a.pts[len(a.pts)-1]}
			if slices.Contains(ends, b.pts[0]) || slices.Contains(ends, b.pts[len(b.pts)-1]) {
				continue
			}
			pts := append(slices.Clone(a.pts), b.pts...)
			if err := requireLoopClearance(ctx, pts,
				[][]int{brepOutAndBack(0, len(a.pts)), brepOutAndBack(len(a.pts), len(b.pts))},
				[]float64{a.sag, b.sag}); err != nil {
				return err
			}
		}
	}
	return nil
}

// brepOutAndBack indexes an open polyline of n points starting at base as a
// closed loop that runs out to its end and back.
func brepOutAndBack(base, n int) []int {
	out := make([]int, 0, 2*n)
	for i := range n {
		out = append(out, base+i)
	}
	for i := n - 2; i >= 1; i-- {
		out = append(out, base+i)
	}
	return out
}
