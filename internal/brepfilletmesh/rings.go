// Package brepfilletmesh places BRep fillet rings and emits their bounded mesh patches.
package brepfilletmesh

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Rings holds one fillet band's interior rings and their proof terms.
type Rings struct {
	Count    int
	Match    []int
	RingV    [][]int
	Dev      [][]float64
	PatchEps map[*topo.Face]float64
	SeamGap  []float64
	Cells    []tessellation.FilletCell
	Patches  int
}

// Input is the recorded band and the rings already shared with adjacent faces.
// Loop, SideV and CapV are read in the wall-to-ring placement order.
// RootAxis marks a band whose patches turn about the side ring's plan points
// at F's level, not about the cap ring's plan points at the side level: a
// boss shell's root transition, whose axis is the source root edge
// (agent-docs/modify-general-design.md §3.1a). SideDelta is the side level's
// displacement.
type Input struct {
	Band        breprecord.LoopBand
	Face        breprecord.Face
	Payload     capblend.Payload
	Loop        *capblend.LoopMesh
	Start       bool
	SideZ       float64
	SideDelta   float64
	SideV, CapV []int
	Curved      []bool
	RootAxis    bool
}

// loopLayout pairs lm's cap samples with its side samples and lists the
// band's cells (tessellation.FilletLoopLayout).
func loopLayout(lm *capblend.LoopMesh) ([]int, []tessellation.FilletCell, int, error) {
	return tessellation.FilletLoopLayout(tessellation.FilletLoopInput{
		Walks: lm.Walks, Joins: capblend.SampleJoins(lm.Joins),
		Count: lm.Count, ArcCount: lm.ArcCount, SideStart: lm.SideStart,
		CapSamples: len(lm.CapPts),
	})
}

// TwistRingCount is the interior ring count of a full-loop fillet band of
// the given radius chorded at chord: tessellation.FilletRingCount's sagitta
// count, raised until every cell's |T|/4 twist bound
// (proofbound.CellTwistOffsetUpper) alone fits the other half of chord, up to
// tessellation.MaxFilletRings. Emit charges each cell the smaller of that
// bound and its height bound (cellOffsetUpper).
//
// Ring k's plan point at cap sample c is side + D·g(φ_k), with D the cap
// sample's offset from its matched side sample, g(φ) = 1 − cos φ (cos φ on a
// root-axis band, whose rings are exchanged) at every sample, and every ring
// vertex of ring k at one level (agent-docs/loop-fillet-design.md §7.1). A
// cell's twist vector between rings k and k+1 is then
// (D(C0) − D(C1))·(g(φ_k) − g(φ_{k+1})), and |Δg| ≤ Δφ = (π/2)/n, so its
// |T|/4 is at most |D(C0) − D(C1)|·π/(8n). A cell from one line–line miter
// to the next along a straight walk has the largest spread: two plan offsets
// along different bisectors. The count only selects the mesh; Emit charges
// the cells the placed rings actually make.
func TwistRingCount(lm *capblend.LoopMesh, radius, chord float64) (int, error) {
	n := tessellation.FilletRingCount(radius, chord)
	match, cells, _, err := loopLayout(lm)
	if err != nil {
		return 0, err
	}
	offset := func(c int) (float64, float64) {
		side := lm.SidePts[match[c]]
		return lm.CapPts[c].U - side.U, lm.CapPts[c].V - side.V
	}
	spread := 0.0
	for _, cell := range cells {
		u0, v0 := offset(cell.C0)
		u1, v1 := offset(cell.C1)
		spread = max(spread, math.Hypot(u0-u1, v0-v1))
	}
	return twistRings(n, spread, chord), nil
}

// twistRings raises the sagitta count n to ⌈spread·π/(4·chord)⌉, the count
// whose |T|/4 bound spread·π/(8n) fits half of chord, capped at
// tessellation.MaxFilletRings. A chord that is not positive, NaN included,
// keeps n, as tessellation.FilletRingCount does.
func twistRings(n int, spread, chord float64) int {
	if !(chord > 0) {
		return n
	}
	need := math.Ceil(spread * math.Pi / (4 * chord))
	if !(need > float64(n)) {
		return n
	}
	return int(min(need, tessellation.MaxFilletRings))
}

// Place adds interior rings after the side and cap rings have their shared vertices.
func (fr *Rings) Place(in Input, e brepgeom.Embed,
	addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	lm := in.Loop
	match, cells, patches, err := loopLayout(lm)
	if err != nil {
		return err
	}
	n := fr.Count
	side := make([]sectionrecord.Point2, len(match))
	for c, index := range match {
		side[c] = lm.SidePts[index]
	}
	ring := tessellation.FilletRingInput{
		Side: side, Cap: lm.CapPts, Radius: in.Band.Setback.Dc,
		RadiusDelta: in.Band.Setback.DcDelta, CapLevel: in.Face.Z0,
		CapLevelDelta: in.Face.Z0Delta, SideLevel: in.SideZ,
		MaterialSign: in.Band.MatSign(in.Face), Count: n,
		Noun: "a fillet band", RingNoun: "a fillet",
	}
	if in.RootAxis {
		if len(in.Curved) > 0 {
			return fmt.Errorf(`%w: a root-axis band has no curved miter`, decaderr.ErrUnsupported)
		}
		// The same quarter circle with its two rings exchanged: its centre
		// is the side point at F's level, so the exchanged reading walks it
		// from the cap ring at F's level to the side ring at the side level.
		ring.Side, ring.Cap = lm.CapPts, side
		ring.SideLevel, ring.CapLevel, ring.CapLevelDelta = in.Face.Z0, in.SideZ, in.SideDelta
		ring.MaterialSign = -ring.MaterialSign
	}
	points, err := tessellation.FilletRingGeometry(ring)
	if err != nil {
		return err
	}
	if in.RootAxis {
		// Ring k counts from the side ring; the exchanged reading counts
		// from the cap ring.
		slices.Reverse(points[1:n])
	}
	if len(in.Curved) > 0 {
		rIv, err := filletband.RadiusInterval(in.Band.Setback.Dc, in.Band.Setback.DcDelta)
		if err != nil {
			return err
		}
		fr.SeamGap, err = tessellation.CurvedMiterRings(points, tessellation.CurvedMiterInput{
			Walks: lm.Walks, Curved: in.Curved, CapWallStart: lm.CapWallStart,
			Radius: rIv, HeldRadius: in.Band.Setback.Dc,
			CapLevel: in.Face.Z0, CapLevelDelta: in.Face.Z0Delta,
			MaterialSign: in.Band.MatSign(in.Face),
		})
		if err != nil {
			return err
		}
	}
	N := len(lm.CapPts)
	if len(fr.SeamGap) == 0 {
		fr.SeamGap = make([]float64, len(lm.Walks))
	}
	fr.Match = match
	fr.Cells, fr.Patches = cells, patches
	fr.RingV = make([][]int, n+1)
	fr.Dev = make([][]float64, n+1)
	fr.RingV[0] = make([]int, N)
	for c := range N {
		fr.RingV[0][c] = in.SideV[match[c]]
	}
	fr.RingV[n] = in.CapV
	for k := 1; k < n; k++ {
		fr.RingV[k] = make([]int, N)
		fr.Dev[k] = make([]float64, N)
		for c, point := range points[k] {
			fr.Dev[k][c] = point.Delta
			fr.RingV[k][c] = addVertex(e.Canon(point.Point.U, point.Point.V, point.Z),
				proofbound.WalkEndBound{U: point.Delta, V: point.Delta})
		}
	}
	return nil
}

// Emit writes the band's strips and their patch bounds. The caller resolves
// every patch role before the first triangle, as the full-band path requires.
func (fr *Rings) Emit(in Input, mesh *topo.Mesh,
	faceOfRole func(string) (*topo.Face, error), bump func(*topo.Face, float64)) error {
	lm := in.Loop
	cells, patches := fr.Cells, fr.Patches
	cbp := in.Payload
	matSign, capZ := 1.0, cbp.Z0
	if !in.Start {
		matSign, capZ = -1, cbp.Z1
	}
	setback := cbp.SetbackAt(matSign)
	delta := cbp.BandDelta[capblend.BandKey{Loop: 0, Start: in.Start}]
	levelDelta := proofbound.AbsSumUpper(setback.DsDelta, proofarith.AddRoundError(capZ, matSign*setback.Ds, in.SideZ))
	axial := cbp.CapBandLevel(capZ, matSign).Bound

	r := setback.AxialUpper()
	dphi := proofbound.UpRound(proofbound.UpRound(math.Pi/2*(1+1e-12)) / float64(fr.Count))
	sPhi := proofbound.ProductUpper(r, proofbound.UpRound(proofbound.ProductUpper(dphi, dphi)/8))

	faces := make([]*topo.Face, patches)
	twist := make([]float64, patches)
	first := make([]int, patches)
	last := make([]int, patches)
	for p := range patches {
		face, err := faceOfRole(in.Band.PatchRole(p))
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
			first[p] = len(mesh.Triangles)
		}
		topobuild.AddTriangle(mesh, [3]int{a, b, c}, faces[p])
		last[p] = len(mesh.Triangles)
	}
	verts := func(k, c int) r3.Vec { return mesh.Vertices[fr.RingV[k][c]] }
	for _, cell := range cells {
		for k := range fr.Count {
			a, b := fr.RingV[k][cell.C0], fr.RingV[k][cell.C1]
			A, B := fr.RingV[k+1][cell.C0], fr.RingV[k+1][cell.C1]
			if in.Start {
				tri(A, B, b, cell.Patch)
				tri(A, b, a, cell.Patch)
			} else {
				tri(a, b, B, cell.Patch)
				tri(a, B, A, cell.Patch)
			}
			if a != b {
				twist[cell.Patch] = math.Max(twist[cell.Patch],
					cellOffsetUpper(twist[cell.Patch], verts(k, cell.C0), verts(k, cell.C1), verts(k+1, cell.C0), verts(k+1, cell.C1)))
			}
		}
	}

	fr.PatchEps = map[*topo.Face]float64{}
	n := len(lm.Walks)
	p := 0
	for i := range n {
		ring := 0.0
		if lm.Walks[i].IsCircular() {
			ring = math.Max(lm.SideSag[i], lm.CapSag[i])
		}
		eps := proofbound.AbsSumUpper(sPhi, ring, twist[p], fr.SeamGap[i], fr.SeamGap[(i+1)%n])
		fr.FinishPatch(mesh, faces[p], eps, delta, levelDelta, axial, first[p], last[p], bump)
		p++
		if ni := (i + 1) % n; lm.Joins != nil && lm.Joins[ni].Arc {
			eps := proofbound.AbsSumUpper(sPhi, lm.ArcSag[ni], twist[p])
			fr.FinishPatch(mesh, faces[p], eps, delta, levelDelta, axial, first[p], last[p], bump)
			p++
		}
	}
	return nil
}

// cellOffsetUpper bounds how far one cell's held triangle pair strays from
// the bilinear patch over its four held corners: the smaller of the twist
// bound |T|/4 and, when the cell projects onto the plane of three corners as
// a strictly convex quadrilateral, the fourth corner's height over that plane
// (proofbound.CellHeightOffsetUpper; agent-docs/loop-fillet-design.md §7.1).
// Both bound the same distance, so their minimum does. A cell whose |T|/4 is
// at most floor, the patch's running maximum, cannot raise that maximum, and
// skips the exact height reading.
func cellOffsetUpper(floor float64, vLo, vHi, wLo, wHi r3.Vec) float64 {
	twist := proofbound.CellTwistOffsetUpper(vLo, vHi, wLo, wHi)
	if !(twist > floor) {
		return twist
	}
	if height, ok := proofbound.CellHeightOffsetUpper(vLo, vHi, wLo, wHi); ok {
		return math.Min(twist, height)
	}
	return twist
}

// FinishPatch records one patch's face displacement and area slack.
func (fr *Rings) FinishPatch(mesh *topo.Mesh, face *topo.Face,
	eps, delta, levelDelta, axial float64, first, last int, bump func(*topo.Face, float64)) {
	fr.PatchEps[face] = eps
	bump(face, proofbound.AbsSumUpper(eps, delta, levelDelta, axial))
	if first < 0 {
		return
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
		tessellation.FilletMeshAreaDeficit(mesh.Vertices, mesh.Triangles[first:last], face.Area, face.AreaBound))
}

// ArcMotion fills motion for reflex connector cap samples.
func (fr *Rings) ArcMotion(lm *capblend.LoopMesh, faceDelta, radiusDelta float64) {
	for i, start := range lm.CapArcStart {
		if start < 0 {
			continue
		}
		for j := start; j < start+lm.ArcCount[i]; j++ {
			lm.CapMotion[j] = proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(lm.CapBound[j]), radiusDelta, faceDelta)
		}
	}
}
