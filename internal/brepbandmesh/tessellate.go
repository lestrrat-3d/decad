package brepbandmesh

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/brepwallmesh"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/units"
)

// Tessellate meshes body, whose payload is bp (docs/general-boolean-design.md
// §4.4). Every swept face chords its own wall once with tessellation.SampleLoop,
// the sampler a prism's walls use, and emits its rim polylines at both levels.
// Every planar face triangulates its loops with triangulation.Triangulate, as a
// prism cap does, over the polylines of the edges it shares, so every edge is
// chorded from one sample set and the mesh closes by construction;
// RequireClosedMesh proves it. A mesh vertex is keyed by its reference
// coordinates (breprecord.Embeds), which a signed permutation states exactly.
// Each record face resolves to the body's face whose origins carry its role.
// verifyAll decides whether the area-slack and occupied-volume proofs at the
// end of the build are composed at all; every audit and every face bound runs
// at each level (docs/tessellation-design.md §1).
func Tessellate(ctx context.Context, body *topo.Body, bp breprecord.Payload, chord float64, verifyAll bool) (*topo.Mesh, error) {
	btopo, err := breprecord.TopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	// The largest section displacement is reserved from the chord budget, as
	// internal/prismtess reserves its own.
	section := bp.SectionDelta()
	budget := chord
	if section > 0 {
		budget = freeform.DownRound(freeform.DownRound(chord - section))
		if budget <= 0 {
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s`,
				decaderr.ErrUnsupported, units.Millimeters(chord), units.Millimeters(section))
		}
	}
	byRole := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			byRole[origin.Role] = face
		}
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		face := byRole[role]
		if face == nil {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, decaderr.ErrDegenerate, role)
		}
		return face, nil
	}
	faceOf := func(fi int) (*topo.Face, error) { return faceOfRole(bp.Faces[fi].Role) }

	bands, imposed, err := Chord(ctx, bp, btopo, budget)
	if err != nil {
		return nil, err
	}

	refView := bp.RefView()
	mesh := new(topo.Mesh)
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
		mesh.Vertices = append(mesh.Vertices, v)
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
		wm, err := brepwallmesh.ChordWall(ctx, f, btopo.Walls[fi], btopo.Embeds[fi], face, budget, work, samples, addVertex)
		if err != nil {
			return nil, err
		}
		walls[fi] = wm
		for _, ui := range btopo.FaceUses[fi] {
			// A rim on a band's side contour is open: the band, not another
			// face, meets it, and no edge holds its polyline.
			switch u := btopo.Uses[ui]; u.Part {
			case brepgeom.Rim0:
				if ei := btopo.EdgeOf[ui]; ei >= 0 {
					edgePoly[ei] = wm.Bottom
				}
			case brepgeom.Rim1:
				if ei := btopo.EdgeOf[ui]; ei >= 0 {
					edgePoly[ei] = wm.Top
				}
			case brepgeom.Side0, brepgeom.Side1:
				// The piece's two vertices are the wall's own row ends at
				// its levels, keyed by the same reference coordinates.
				edgePoly[btopo.EdgeOf[ui]] = []int{addVertex(u.DirFrom, proofbound.WalkEndBound{}), addVertex(u.DirTo, proofbound.WalkEndBound{})}
			}
		}
		for j := 0; j+1 < len(wm.Cols); j++ {
			tessellation.ZipColumns(wm.Cols[j], wm.Cols[j+1], wm.Levels[j], wm.Levels[j+1], func(tri [3]int) {
				topobuild.AddTriangle(mesh, tri, face)
			})
		}
		faceTrim[face] = wm.Sag
		faceAxial[face] = proofbound.AbsSumUpper(math.Max(f.Z0Delta, f.Z1Delta), f.Delta)
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, wm.WallSlack)
	}
	if err := brepwallmesh.WallClearance(ctx, bp, btopo, walls, canon); err != nil {
		return nil, err
	}

	bandsOf, terminalPoly, err := Place(bands, btopo, addVertex, &canon)
	if err != nil {
		return nil, err
	}

	// faceCapSlack is each planar face's own share of its curved edges'
	// chord-versus-arc area: one cap's share per curved edge it bounds.
	faceCapSlack := map[*topo.Face]float64{}
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
		var pts []sectionrecord.Point2
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
				ring, sag, slack, ok := OpenRing(bp, bands, terminalPoly, fi, ui, u)
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
				pts = append(pts, sectionrecord.Point2{U: local[0], V: local[1]})
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
		if err := tessellation.RequireLoopClearance(ctx, pts, loops, loopSag); err != nil {
			return nil, err
		}
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		if err != nil {
			return nil, err
		}
		for _, tri := range tris {
			a, b, c := meshIdx[tri[0]], meshIdx[tri[1]], meshIdx[tri[2]]
			if !f.Outward {
				b, c = c, b
			}
			topobuild.AddTriangle(mesh, [3]int{a, b, c}, face)
		}
		faceTrim[face] = trim
		faceAxial[face] = proofbound.AbsSumUpper(f.Z0Delta, f.Delta)
		faceCapSlack[face] = capSlack
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, capSlack)
	}
	if len(bands) > 0 {
		bump := func(f *topo.Face, v float64) { faceTrim[f] = math.Max(faceTrim[f], v) }
		if err := Emit(ctx, bands, bp, mesh, faceOfRole, bump); err != nil {
			return nil, err
		}
	}
	if bp.Xform.IsReflection() {
		for i := range mesh.Triangles {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(mesh.Triangles)); err != nil {
		return nil, err
	}
	if len(bands) > 0 {
		// A band's strips and fans can collapse where its rings meet; the cap
		// blend mesh build refuses the same two faults (capblend.TessellateMesh).
		if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
			return nil, err
		}
		if err := tessellation.RequireCapBlendFacetAreas(mesh.Vertices, mesh.Triangles, "chamfered brep"); err != nil {
			return nil, err
		}
	}
	storeMax, err := tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, faceTrim, faceAxial, store, 0)
	if err != nil {
		return nil, err
	}
	for f, bound := range bounds {
		topobuild.SetFaceBound(mesh, f, bound)
	}
	if !verifyAll {
		return mesh, nil
	}

	// Occupied volume (tessellation §5's prism term applied per face): each
	// curved wall's circular-segment slivers over its height, each wall's
	// section band over its height, each planar face's level displacement over
	// its area, and every computed coordinate's swept volume. Absolute sums
	// throughout: an occupied-volume bound admits no cancellation.
	areaUpper := tessellation.FaceAreaUpper(mesh.Vertices, mesh.Triangles, mesh.Source, store)
	var terms []float64
	for fi, f := range bp.Faces {
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		if f.Planar() {
			region := tessellation.BrepPlanarDisplacement(f.Delta, btopo.Planar[fi])
			mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, region)
			terms = append(terms, proofbound.ProductUpper(f.Z0Delta,
				proofbound.AbsSumUpper(areaUpper[face], faceCapSlack[face], region)))
			continue
		}
		w := btopo.Walls[fi]
		height := proofbound.UpRound(f.Z1 - f.Z0)
		moved := proofbound.AbsSumUpper(height, f.Z0Delta, f.Z1Delta)
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
			proofbound.ProductUpper(proofbound.SectionDisplacementLength(f.Delta, 1), moved))
		terms = append(terms,
			proofbound.ProductUpper(height, walls[fi].SegmentArea),
			proofbound.ProductUpper(moved, proofbound.SectionDisplacementArea(f.Delta, 1, proofbound.AbsSumUpper(w.Length, w.LengthBound))))
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
		tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, store))
	if len(bands) > 0 {
		// A band the cap-loop chamfer's occupied-volume proof does not admit
		// leaves the mesh export-only (docs/tessellation-reach-design.md §7),
		// and the mesh boolean refuses it with the same reason.
		refusal, err := OccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), bp)
		if err != nil {
			return nil, err
		}
		if refusal != nil {
			mesh.SymDiffOK = false
			return mesh, nil //nolint:nilerr // refusal is the admission result; err was checked above
		}
		motion, err := Motion(ctx, bands, store, round)
		if err != nil {
			return nil, err
		}
		if storeMax, err = tessellation.StoreMax(motion); err != nil {
			return nil, err
		}
		terms = append(terms, VolumeTerms(bands, areaUpper)...)
	}
	terms = append(terms, proofbound.SweptVolumeAllow(storeMax,
		proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, storeMax)))
	total, err := tessellation.SymDiff(terms)
	if err != nil {
		return nil, err
	}
	mesh.VolSymDiff = total
	mesh.SymDiffOK = true
	return mesh, nil
}
