package brepbandmesh

import (
	"context"
	"maps"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepfilletmesh"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// Place adds every band's ring vertices once every wall has placed its
// samples: a wall beside a band took the band's own side-ring points, so the
// two name the same vertices. addVertex keys each vertex by its reference
// coordinates and appends a new one to *canon, which a selected band reads
// right after its own rings are placed. Place returns, per planar face, the
// indices of the bands whose cap contour is a whole loop of that face, and,
// per open use a selected band closes, that use's vertex polyline.
func Place(bands []Band, btopo *breprecord.Topology,
	addVertex func([3]float64, proofbound.WalkEndBound) int, canon *[][3]float64) (map[int][]int, map[int][]int, error) {
	bandsOf := map[int][]int{}
	terminalPoly := map[int][]int{}
	for bi := range bands {
		bc := &bands[bi]
		e := btopo.Embeds[bc.band.Face]
		bc.place(e, addVertex)
		if bc.fillet != nil {
			if err := bc.placeRings(e, addVertex); err != nil {
				return nil, nil, err
			}
		}
		if bc.partial != nil {
			polys, err := bc.partialOpenPolys(btopo, *canon)
			if err != nil {
				return nil, nil, err
			}
			maps.Copy(terminalPoly, polys)
			continue
		}
		if !bc.bossShell && !bc.pocketShell {
			bandsOf[bc.band.Face] = append(bandsOf[bc.band.Face], bi)
		}
	}
	return bandsOf, terminalPoly, nil
}

// place adds both rings' vertices: the side ring at the side level and the
// cap contour ring at F's level, each sample keyed by its reference
// coordinates so a wall's rim sample at the same point is the same vertex.
func (bc *Band) place(e brepgeom.Embed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	if bc.partial != nil {
		bc.placePartial(e, addVertex)
		return
	}
	for j, p := range bc.lm.SidePts {
		bc.sideV = append(bc.sideV, addVertex(e.Canon(p.U, p.V, bc.sideZ), bc.lm.SideBound[j]))
	}
	for j, p := range bc.lm.CapPts {
		bc.capV = append(bc.capV, addVertex(e.Canon(p.U, p.V, bc.face.Z0), bc.lm.CapBound[j]))
	}
}

// placePartial adds a selected band's side and cap vertices column by column.
func (bc *Band) placePartial(e brepgeom.Embed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	for _, col := range bc.partial.Columns {
		s, c := col.Side, col.Cap
		bc.sideV = append(bc.sideV, addVertex(e.Canon(s.U, s.V, bc.sideZ), col.SideBound))
		bc.capV = append(bc.capV, addVertex(e.Canon(c.U, c.V, bc.face.Z0), col.CapBound))
	}
}

func (bc *Band) placeRings(e brepgeom.Embed,
	addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	if bc.partial != nil {
		return bc.fillet.PlacePartial(bc.filletInput(), bc.partial, e, addVertex)
	}
	return bc.fillet.Place(bc.filletInput(), e, addVertex)
}

func (bc *Band) partialOpenPolys(btopo *breprecord.Topology, canon [][3]float64) (map[int][]int, error) {
	ringV := [][]int{bc.sideV, bc.capV}
	if bc.fillet != nil {
		ringV = bc.fillet.RingV
	}
	return brepfilletmesh.OpenPolys(bc.partial, bc.band, btopo, canon, bc.sideZ, ringV)
}

// Loop is the index of the loop the band's cap contour bounds on its face.
func (bc *Band) Loop() int { return bc.band.Loop }

// CapRing is the band's cap contour ring on its own face: the ring's
// plane points and their mesh vertices (a fillet band's ring compacted), the
// ring's largest sagitta, and its chord-versus-arc area.
func (bc *Band) CapRing() ([]sectionrecord.Point2, []int, float64, float64) {
	proof := bc.lm.Proof()
	sag := tessellation.CapBlendRingSagitta(proof, true)
	capPts, capV := bc.lm.CapPts, bc.capV
	if bc.cbp.Fillet {
		capPts, capV = tessellation.CompactCapBlendRing(capPts, capV)
	}
	return capPts, capV, sag, tessellation.CapBlendRingSegmentArea(proof, true, bc.band.Setback.Dc)
}

// OpenRing is the band ring that closes open use ui (u) of planar face fi:
// the round boss shell's side ring on its ledge's hole, the round pocket
// shell's cap ring on its floor, or the polyline a selected band's Place
// returned in terminal. It returns the polyline, its sagitta and its
// chord-versus-arc area; ok is false when no band ring closes the use.
func OpenRing(bp breprecord.Payload, bands []Band, terminal map[int][]int, fi, ui int, u brepgeom.Use) ([]int, float64, float64, bool) {
	if bp.BossShell != nil && bp.BossShell.Round != nil &&
		fi == bp.BossShell.LedgeFace && u.Loop == 1 {
		bc := &bands[len(bp.LoopBands)]
		poly := append(slices.Clone(bc.sideV), bc.sideV[0])
		slices.Reverse(poly) // the ledge walks this hole clockwise
		return poly, bc.lm.SideSag[0], tessellation.WalkSegmentArea(bc.lm.Walks[0].SegmentWalk, bc.lm.Count[0]), true
	}
	if bp.PocketShell != nil && bp.PocketShell.Round != nil &&
		fi == bp.PocketShell.FloorFace && u.Loop == 0 {
		bc := &bands[len(bp.LoopBands)]
		poly := append(slices.Clone(bc.capV), bc.capV[0])
		return poly, bc.lm.CapSag[0], tessellation.WalkSegmentArea(u.Walk, bc.lm.Count[0]), true
	}
	poly := terminal[ui]
	if poly == nil {
		return nil, 0, 0, false
	}
	sag := tessellation.ChordSagitta(u.Walk.Radius, math.Abs(u.Walk.Th1-u.Walk.Th0), len(poly)-1)
	return poly, sag, tessellation.WalkSegmentArea(u.Walk, len(poly)-1), true
}

// Emit writes every band's patch cells into mesh. faceOfRole resolves a
// patch role to the body's face, and bump raises a face's trim bound.
func Emit(ctx context.Context, bands []Band, bp breprecord.Payload, mesh *topo.Mesh,
	faceOfRole func(string) (*topo.Face, error), bump func(*topo.Face, float64)) error {
	geomOf := func(bi int) map[string]capband.Patch {
		out := map[string]capband.Patch{}
		if bi < len(bp.LoopPatches) {
			for _, p := range bp.LoopPatches[bi] {
				out[p.Role] = p.Geom
			}
		}
		return out
	}
	wb := proofbound.NewWorkBudget(ctx)
	for bi := range bands {
		if err := bands[bi].emit(wb, mesh, geomOf(bi), faceOfRole, bump); err != nil {
			return err
		}
	}
	return nil
}

// emit writes the band's patch cells. A band whose walls rise off its face
// (sigma +1) fills a concave corner rather than removing a wedge, and its
// patches face the other way (breprecord's attachLoopBands), so its windings turn over.
func (bc *Band) emit(budget *proofbound.WorkBudget, m *topo.Mesh, geom map[string]capband.Patch,
	faceOfRole func(string) (*topo.Face, error), bump func(*topo.Face, float64)) error {
	if bc.partial != nil {
		first := len(m.Triangles)
		var err error
		if bc.band.Kind == breprecord.BandFillet {
			err = bc.fillet.EmitPartialWithLevel(bc.filletInput(), bc.partial, m, bc.sideDelta, faceOfRole, bump)
		} else {
			err = bc.emitPartialChamfer(m, faceOfRole, bump)
		}
		if err != nil {
			return err
		}
		if bc.band.Sigma > 0 {
			flipWindings(m, first)
		}
		return nil
	}
	if bc.fillet != nil {
		first := len(m.Triangles)
		if err := bc.fillet.Emit(bc.filletInput(), m, faceOfRole, bump); err != nil {
			return err
		}
		// The pocket uses an upward virtual cap to generate the band, while
		// its recorded square underside faces down. Its generated winding
		// already opposes both neighbouring faces.
		if bc.band.Sigma > 0 && !bc.pocketShell {
			flipWindings(m, first)
		}
		return nil
	}
	lm := &bc.lm
	if bc.start {
		lm.SideLo, lm.CapLoV = bc.sideV, bc.capV
	} else {
		lm.SideHi, lm.CapHiV = bc.sideV, bc.capV
	}
	first := len(m.Triangles)
	if err := capblend.EmitBand(budget, m, bc.cbp, lm, bc.start, faceOfRole, geom, bump, bc.band.PatchRole); err != nil {
		return err
	}
	if bc.band.Sigma > 0 {
		flipWindings(m, first)
	}
	return nil
}

// flipWindings turns over every triangle from index first on.
func flipWindings(m *topo.Mesh, first int) {
	for i := first; i < len(m.Triangles); i++ {
		m.Triangles[i][1], m.Triangles[i][2] = m.Triangles[i][2], m.Triangles[i][1]
	}
}

func (bc *Band) emitPartialChamfer(m *topo.Mesh, faceOfRole func(string) (*topo.Face, error),
	bump func(*topo.Face, float64)) error {
	for p := range bc.partial.PatchSag {
		face, err := faceOfRole(bc.band.PatchRole(p))
		if err != nil {
			return err
		}
		first := len(m.Triangles)
		for _, cell := range bc.partial.Cells {
			if cell.Patch != p {
				continue
			}
			a, b := bc.sideV[cell.C0], bc.sideV[cell.C1]
			A, B := bc.capV[cell.C0], bc.capV[cell.C1]
			if bc.start {
				topobuild.AddTriangle(m, [3]int{A, B, b}, face)
				topobuild.AddTriangle(m, [3]int{A, b, a}, face)
			} else {
				topobuild.AddTriangle(m, [3]int{a, b, B}, face)
				topobuild.AddTriangle(m, [3]int{a, B, A}, face)
			}
		}
		// The exact rational coplanarity gate and zero contour/level
		// displacement make the patch deviation zero. Stored vertices still
		// contribute to the body's motion and area allowances.
		bump(face, 0)
		m.AreaSlack = proofbound.AbsSumUpper(m.AreaSlack,
			tessellation.FilletMeshAreaDeficit(m.Vertices, m.Triangles[first:], face.Area, face.AreaBound))
	}
	return nil
}
