package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepbandmesh"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/brepwallmesh"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/units"
)

// tessellateBrep meshes a brepPayload (docs/general-boolean-design.md §4.4).
// Every swept face chords its own wall once with tessellation.SampleLoop, the
// sampler a prism's walls use, and emits its rim polylines at both levels.
// Every planar face triangulates its loops with triangulation.Triangulate, as a
// prism cap does, over the polylines of the edges it shares, so every edge is
// chorded from one sample set and the mesh closes by construction;
// RequireClosedMesh proves it. A mesh vertex is keyed by its reference
// coordinates (breprecord.Embeds), which a signed permutation states exactly.
// It stays in root because it resolves the body's Face handles by role and
// builds the Mesh handle; the route L bands are internal/brepbandmesh's.
func tessellateBrep(ctx context.Context, b *Body, bp brepPayload, chord float64, verify Verification) (*Mesh, error) {
	btopo, err := breprecord.TopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	// The largest section displacement is reserved from the chord budget, as
	// tessellatePrism reserves its own.
	section := bp.SectionDelta()
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
		face := byRole[bp.Faces[fi].Role]
		if face == nil {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, bp.Faces[fi].Role)
		}
		return face, nil
	}

	bands, imposed, err := brepbandmesh.Chord(ctx, bp, btopo, budget)
	if err != nil {
		return nil, err
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		face := byRole[role]
		if face == nil {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return faceNode(face), nil
	}

	refView := bp.RefView()
	mesh := bornMesh(new(Mesh))
	var store, round []float64
	var canon [][3]float64
	index := map[[3]float64]int{}
	addVertex := func(c [3]float64, sample proofbound.WalkEndBound) int {
		allow := proofbound.WalkEndBoundAllow(sample)
		if i, ok := index[c]; ok {
			store[i] = math.Max(store[i], proofbound.AbsSumUpper(allow, round[i]))
			return i
		}
		v := refView.Point(c[0], c[1], c[2])
		r := refView.PointRound(c[0], c[1], c[2], v)
		mesh.n.Vertices = append(mesh.n.Vertices, v)
		round = append(round, r)
		store = append(store, proofbound.AbsSumUpper(allow, r))
		canon = append(canon, c)
		index[c] = len(canon) - 1
		return len(canon) - 1
	}
	faceTrim := map[*topo.Face]float64{}
	faceAxial := map[*topo.Face]float64{}
	edgePoly := make([][]int, len(btopo.Edges))
	walls := map[int]brepwallmesh.WallMesh{}
	work := freeform.NewFreeformWork()
	for fi, f := range bp.Faces {
		if f.Planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		var samples *tessellation.ChordSamples[*topo.Face]
		if s, ok := imposed[fi]; ok {
			samples = &s
		}
		wm, err := brepwallmesh.ChordWall(ctx, f, btopo.Walls[fi], btopo.Embeds[fi], faceNode(face), budget, work, samples, addVertex)
		if err != nil {
			return nil, err
		}
		walls[fi] = wm
		for _, ui := range btopo.FaceUses[fi] {
			// A rim on a band's side contour is open: the band, not another
			// face, meets it, and no edge holds its polyline.
			switch u := btopo.Uses[ui]; u.Part {
			case brepRim0:
				if ei := btopo.EdgeOf[ui]; ei >= 0 {
					edgePoly[ei] = wm.Bottom
				}
			case brepRim1:
				if ei := btopo.EdgeOf[ui]; ei >= 0 {
					edgePoly[ei] = wm.Top
				}
			case brepSide0, brepSide1:
				// The piece's two vertices are the wall's own row ends at
				// its levels, keyed by the same reference coordinates.
				edgePoly[btopo.EdgeOf[ui]] = []int{addVertex(u.DirFrom, proofbound.WalkEndBound{}), addVertex(u.DirTo, proofbound.WalkEndBound{})}
			}
		}
		for j := 0; j+1 < len(wm.Cols); j++ {
			tessellation.ZipColumns(wm.Cols[j], wm.Cols[j+1], wm.Levels[j], wm.Levels[j+1], func(tri [3]int) {
				topobuild.AddTriangle(&mesh.n, tri, faceNode(face))
			})
		}
		faceTrim[faceNode(face)] = wm.Sag
		faceAxial[faceNode(face)] = proofbound.AbsSumUpper(math.Max(f.Z0Delta, f.Z1Delta), f.Delta)
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, wm.WallSlack)
	}
	if err := brepwallmesh.WallClearance(ctx, bp, btopo, walls, canon); err != nil {
		return nil, err
	}

	bandsOf, terminalPoly, err := brepbandmesh.Place(bands, btopo, addVertex, &canon)
	if err != nil {
		return nil, err
	}

	// faceCapSlack is each planar face's own share of its curved edges'
	// chord-versus-arc area: one cap's share per curved edge it bounds.
	faceCapSlack := map[*Face]float64{}
	for fi, f := range bp.Faces {
		if !f.Planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		e := btopo.Embeds[fi]
		loops := make([][]int, len(btopo.Planar[fi]))
		loopSag := make([]float64, len(loops))
		var pts []planePoint
		var meshIdx []int
		trim, capSlack := 0.0, 0.0
		ringLoop := map[int]int{}
		for _, bi := range bandsOf[fi] {
			ringLoop[bands[bi].Loop()] = bi
		}
		for _, ui := range btopo.FaceUses[fi] {
			u := btopo.Uses[ui]
			ei := btopo.EdgeOf[ui]
			var poly []int
			switch {
			case ei >= 0:
				owner := btopo.Uses[btopo.Edges[ei][0]]
				poly = edgePoly[ei]
				if poly == nil {
					poly = []int{addVertex(owner.DirFrom, owner.StartBound()), addVertex(owner.DirTo, owner.EndBound())}
					edgePoly[ei] = poly
				}
				if !btopo.Forward(ui) {
					poly = slices.Clone(poly)
					slices.Reverse(poly)
				}
				if wm, ok := walls[owner.Face]; ok && brepgeom.IsRim(owner.Part) {
					trim = math.Max(trim, wm.Sag)
					loopSag[u.Loop] = math.Max(loopSag[u.Loop], wm.Sag)
					capSlack = proofbound.AbsSumUpper(capSlack, wm.CapSlack)
				}
			default:
				// An open use is a band's boundary. The band's cap contour
				// on this face is its whole ring, placed below; a segment of
				// a (pl) face on the band's side contour is a straight line
				// between its two held ends.
				if _, ok := ringLoop[u.Loop]; ok {
					continue
				}
				ring, sag, slack, ok := brepbandmesh.OpenRing(bp, bands, terminalPoly, fi, ui, u)
				if !ok {
					poly = []int{addVertex(u.DirFrom, u.StartBound()), addVertex(u.DirTo, u.EndBound())}
					break
				}
				poly = ring
				trim = math.Max(trim, sag)
				loopSag[u.Loop] = math.Max(loopSag[u.Loop], sag)
				capSlack = proofbound.AbsSumUpper(capSlack, slack)
			}
			for _, vi := range poly[:len(poly)-1] {
				local := e.Local(canon[vi])
				pts = append(pts, planePoint{U: local[0], V: local[1]})
				meshIdx = append(meshIdx, vi)
				loops[u.Loop] = append(loops[u.Loop], len(pts)-1)
			}
		}
		for _, bi := range bandsOf[fi] {
			loop := bands[bi].Loop()
			capPts, capV, sag, slack := bands[bi].CapRing()
			for j, p := range capPts {
				pts = append(pts, p)
				meshIdx = append(meshIdx, capV[j])
				loops[loop] = append(loops[loop], len(pts)-1)
			}
			trim = math.Max(trim, sag)
			loopSag[loop] = math.Max(loopSag[loop], sag)
			capSlack = proofbound.AbsSumUpper(capSlack, slack)
		}
		if err := tessellation.RequireLoopClearance(ctx, point2ToRecordSlice(pts), loops, loopSag); err != nil {
			return nil, err
		}
		tris, err := triangulation.Triangulate(ctx, point2ToRecordSlice(pts), loops)
		if err != nil {
			return nil, err
		}
		for _, tri := range tris {
			a, bb, c := meshIdx[tri[0]], meshIdx[tri[1]], meshIdx[tri[2]]
			if !f.Outward {
				bb, c = c, bb
			}
			topobuild.AddTriangle(&mesh.n, [3]int{a, bb, c}, faceNode(face))
		}
		faceTrim[faceNode(face)] = trim
		faceAxial[faceNode(face)] = proofbound.AbsSumUpper(f.Z0Delta, f.Delta)
		faceCapSlack[face] = capSlack
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, capSlack)
	}
	if len(bands) > 0 {
		bump := func(f *topo.Face, v float64) { faceTrim[f] = math.Max(faceTrim[f], v) }
		if err := brepbandmesh.Emit(ctx, bands, bp, &mesh.n, faceOfRole, bump); err != nil {
			return nil, err
		}
	}
	if bp.Xform.IsReflection() {
		for i := range mesh.n.Triangles {
			mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
		}
	}
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(mesh.n.Triangles)); err != nil {
		return nil, err
	}
	if len(bands) > 0 {
		// A band's strips and fans can collapse where its rings meet; the cap
		// blend mesh build refuses the same two faults (capblend.TessellateMesh).
		if err := tessellation.RequireVertexLinks(ctx, len(mesh.n.Vertices), mesh.n.Triangles); err != nil {
			return nil, err
		}
		if err := tessellation.RequireCapBlendFacetAreas(mesh.n.Vertices, mesh.n.Triangles, "chamfered brep"); err != nil {
			return nil, err
		}
	}
	storeMax, err := tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	if err := composeFaceBounds(mesh, faceTrim, faceAxial, store); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return mesh, nil
	}

	// Occupied volume (tessellation §5's prism term applied per face): each
	// curved wall's circular-segment slivers over its height, each wall's
	// section band over its height, each planar face's level displacement over
	// its area, and every computed coordinate's swept volume. Absolute sums
	// throughout: an occupied-volume bound admits no cancellation.
	areaUpper := tessellation.FaceAreaUpper(mesh.n.Vertices, mesh.n.Triangles, mesh.n.Source, store)
	var terms []float64
	for fi, f := range bp.Faces {
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		if f.Planar() {
			region := tessellation.BrepPlanarDisplacement(f.Delta, btopo.Planar[fi])
			mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, region)
			terms = append(terms, proofbound.ProductUpper(f.Z0Delta,
				proofbound.AbsSumUpper(areaUpper[faceNode(face)], faceCapSlack[face], region)))
			continue
		}
		w := btopo.Walls[fi]
		height := proofbound.UpRound(f.Z1 - f.Z0)
		moved := proofbound.AbsSumUpper(height, f.Z0Delta, f.Z1Delta)
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
			proofbound.ProductUpper(proofbound.SectionDisplacementLength(f.Delta, 1), moved))
		terms = append(terms,
			proofbound.ProductUpper(height, walls[fi].SegmentArea),
			proofbound.ProductUpper(moved, proofbound.SectionDisplacementArea(f.Delta, 1, proofbound.AbsSumUpper(w.Length, w.LengthBound))))
	}
	mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
		tessellation.StoreAreaAllow(mesh.n.Vertices, mesh.n.Triangles, store))
	if len(bands) > 0 {
		// A band the cap-loop chamfer's occupied-volume proof does not admit
		// leaves the mesh export-only (docs/tessellation-reach-design.md §7),
		// and the mesh boolean refuses it with the same reason.
		refusal, err := brepbandmesh.OccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), bp)
		if err != nil {
			return nil, err
		}
		if refusal != nil {
			mesh.n.SymDiffOK = false
			return mesh, nil //nolint:nilerr // refusal is the admission result; err was checked above
		}
		motion, err := brepbandmesh.Motion(ctx, bands, store, round)
		if err != nil {
			return nil, err
		}
		if storeMax, err = tessellation.StoreMax(motion); err != nil {
			return nil, err
		}
		terms = append(terms, brepbandmesh.VolumeTerms(bands, areaUpper)...)
	}
	terms = append(terms, proofbound.SweptVolumeAllow(storeMax,
		proofbound.PerturbedAreaUpper(mesh.n.Vertices, mesh.n.Triangles, storeMax)))
	if err := publishSymDiff(mesh, terms); err != nil {
		return nil, err
	}
	return mesh, nil
}
