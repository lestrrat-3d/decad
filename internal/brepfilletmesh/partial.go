package brepfilletmesh

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// PlacePartial adds the selected band's interior rings after its boundary rings.
func (fr *Rings) PlacePartial(in Input, partial *partialband.PartialBandLayout,
	e brepgeom.Embed, addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	n, N := fr.Count, len(in.CapV)
	fr.RingV = make([][]int, n+1)
	fr.Dev = make([][]float64, n+1)
	fr.RingV[0], fr.RingV[n] = in.SideV, in.CapV
	side, capPoints := make([]sectionrecord.Point2, N), make([]sectionrecord.Point2, N)
	for c, col := range partial.Columns {
		side[c], capPoints[c] = col.Side, col.Cap
	}
	points, err := tessellation.FilletRingGeometry(tessellation.FilletRingInput{
		Side: side, Cap: capPoints, Radius: in.Band.Setback.Dc,
		RadiusDelta: in.Band.Setback.DcDelta, CapLevel: in.Face.Z0,
		CapLevelDelta: in.Face.Z0Delta, SideLevel: in.SideZ,
		MaterialSign: in.Band.MatSign(in.Face), Count: n,
		Noun: "a partial fillet band", RingNoun: "a partial fillet",
	})
	if err != nil {
		return err
	}
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

// OpenPolys maps selected-band ring vertices onto the record's open uses.
func OpenPolys(partial *partialband.PartialBandLayout,
	band breprecord.LoopBand, btopo *breprecord.Topology, canon [][3]float64,
	sideZ float64, ringV [][]int) (map[int][]int, error) {
	terminals := make([]partialband.Terminal, len(band.Terminals))
	for i, terminal := range band.Terminals {
		terminals[i] = partialband.Terminal{Face: terminal.Face, Loop: terminal.Loop, Seg: terminal.Seg}
	}
	return partialband.OpenPolys(partialband.OpenPolyInput{
		Layout: partial, Face: band.Face, Loop: band.Loop,
		Orig: band.Orig.Segments, CapWalk: band.CapWalk, Terminals: terminals,
		RingV: ringV, Canon: canon, Uses: btopo.Uses, FaceUses: btopo.FaceUses,
		Open: btopo.Open, Embed: btopo.Embeds[band.Face], SideZ: sideZ,
	})
}

// EmitPartialWithLevel resolves each patch role just before that patch's
// triangles and uses the recorded side-level bound.
func (fr *Rings) EmitPartialWithLevel(in Input, partial *partialband.PartialBandLayout,
	mesh *topo.Mesh, sideDelta float64,
	faceOfRole func(string) (*topo.Face, error), bump func(*topo.Face, float64)) error {
	fr.PatchEps = map[*topo.Face]float64{}
	r := in.Band.Setback.AxialUpper()
	dphi := proofbound.UpRound(proofbound.UpRound(math.Pi/2*(1+1e-12)) / float64(fr.Count))
	sPhi := proofbound.ProductUpper(r, proofbound.UpRound(proofbound.ProductUpper(dphi, dphi)/8))
	delta := in.Face.Delta
	levelDelta := proofbound.AbsSumUpper(in.Band.Setback.DsDelta, sideDelta)
	axial := in.Payload.CapBandLevel(in.Face.Z0, in.Band.MatSign(in.Face)).Bound
	for p := range partial.PatchSag {
		face, err := faceOfRole(in.Band.PatchRole(p))
		if err != nil {
			return err
		}
		first := len(mesh.Triangles)
		twist := 0.0
		tri := func(a, b, c int) {
			if a != b && b != c && a != c {
				topobuild.AddTriangle(mesh, [3]int{a, b, c}, face)
			}
		}
		for _, cell := range partial.Cells {
			if cell.Patch != p {
				continue
			}
			for k := range fr.Count {
				a, b := fr.RingV[k][cell.C0], fr.RingV[k][cell.C1]
				A, B := fr.RingV[k+1][cell.C0], fr.RingV[k+1][cell.C1]
				if in.Start {
					tri(A, B, b)
					tri(A, b, a)
				} else {
					tri(a, b, B)
					tri(a, B, A)
				}
				if a != b {
					twist = math.Max(twist, proofbound.CellTwistOffsetUpper(
						mesh.Vertices[a], mesh.Vertices[b], mesh.Vertices[A], mesh.Vertices[B]))
				}
			}
		}
		eps := proofbound.AbsSumUpper(sPhi, partial.PatchSag[p], twist)
		fr.FinishPatch(mesh, face, eps, delta, levelDelta, axial, first, len(mesh.Triangles), bump)
	}
	return nil
}

// PartialMotion propagates side, cap and interpolation displacement to selected rings.
func (fr *Rings) PartialMotion(in Input, sideDelta float64, motion, store, round []float64) {
	PartialBoundaryMotion(in, sideDelta, motion, store)
	for k := 1; k < fr.Count; k++ {
		for c, vi := range fr.RingV[k] {
			ends := math.Max(motion[fr.RingV[0][c]], motion[fr.RingV[fr.Count][c]])
			motion[vi] = proofbound.AbsSumUpper(ends, fr.Dev[k][c], round[vi])
		}
	}
}

// PartialBoundaryMotion propagates side and cap displacement for selected bands.
func PartialBoundaryMotion(in Input, sideDelta float64, motion, store []float64) {
	for _, vi := range in.SideV {
		motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], sideDelta))
	}
	for _, vi := range in.CapV {
		motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], in.Face.Delta, in.Face.Z0Delta))
	}
}
