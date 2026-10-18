// Package brepfilletmesh places BRep fillet rings and emits their bounded mesh patches.
package brepfilletmesh

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
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
type Input struct {
	Band        breprecord.LoopBand
	Face        breprecord.Face
	Payload     capblend.Payload
	Loop        *capblend.LoopMesh
	Start       bool
	SideZ       float64
	SideV, CapV []int
	Curved      []bool
}

// Place adds interior rings after the side and cap rings have their shared vertices.
func (fr *Rings) Place(in Input, e brepgeom.Embed,
	addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	lm := in.Loop
	match, cells, patches, err := tessellation.FilletLoopLayout(tessellation.FilletLoopInput{
		Walks: lm.Walks, Joins: capblend.SampleJoins(lm.Joins),
		Count: lm.Count, ArcCount: lm.ArcCount, SideStart: lm.SideStart,
		CapSamples: len(lm.CapPts),
	})
	if err != nil {
		return err
	}
	n := fr.Count
	side := make([]sectionrecord.Point2, len(match))
	for c, index := range match {
		side[c] = lm.SidePts[index]
	}
	points, err := tessellation.FilletRingGeometry(tessellation.FilletRingInput{
		Side: side, Cap: lm.CapPts, Radius: in.Band.Setback.Dc,
		RadiusDelta: in.Band.Setback.DcDelta, CapLevel: in.Face.Z0,
		CapLevelDelta: in.Face.Z0Delta, SideLevel: in.SideZ,
		MaterialSign: in.Band.MatSign(in.Face), Count: n,
		Noun: "a fillet band", RingNoun: "a fillet",
	})
	if err != nil {
		return err
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
					proofbound.CellTwistOffsetUpper(verts(k, cell.C0), verts(k, cell.C1), verts(k+1, cell.C0), verts(k+1, cell.C1)))
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
