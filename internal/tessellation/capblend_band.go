package tessellation

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// CapBlendBandInput records one band's shared ring indices and proof terms.
type CapBlendBandInput struct {
	Loop                        int
	Start                       bool
	Walks                       []survey2d.SideWalk
	Joins                       []CapBlendJoin
	Count, ArcCount             []int
	SideStart, CapWallStart     []int
	CapArcStart                 []int
	SideV, CapV                 []int
	SideSag, CapSag, CapRadius  []float64
	ArcSag, LocusGap            []float64
	D, Delta, LevelDelta, Axial float64
}

// CapBlendBandPatch is the numeric part of the built patch with one role.
type CapBlendBandPatch struct {
	Circular   bool
	SideRadius float64
	CapRadius  float64
	Skew       float64
	AreaAllow  float64
}

// CapBlendBandMesh lets the tessellator append facets in the root mesh's
// existing order while retaining its face attribution and area accumulator.
type CapBlendBandMesh[F comparable] interface {
	AddTriangle([3]int, F)
	Vertices() []r3.Vec
	Triangles() [][3]int
	Bump(F, float64)
	AddAreaSlack(float64, float64)
}

// EmitCapBlendBand writes apex fans first, then one fixed-diagonal strip per
// wall. Patch returns the built face and its proof inputs by patch index.
func EmitCapBlendBand[F comparable](budget *proofbound.WorkBudget, mesh CapBlendBandMesh[F],
	in CapBlendBandInput, patch func(int) (F, CapBlendBandPatch, error),
) error {
	n := len(in.Walks)
	apexIdx := map[int]int{}
	next := 0
	for i := range n {
		if in.Joins != nil && in.Joins[i].Arc {
			apexIdx[i] = next
			next++
		}
	}

	for i := range n {
		p, isApex := apexIdx[i]
		if !isApex {
			continue
		}
		if err := budget.Step(); err != nil {
			return err
		}
		face, g, err := patch(p)
		if err != nil {
			return err
		}
		if !g.Circular || g.SideRadius != 0 {
			return fmt.Errorf(`%w: patch %d of the chamfer band on loop %d is not the apex patch its corner states`, decaderr.ErrDegenerate, p, in.Loop)
		}
		apex := in.SideV[in.SideStart[i]]
		base := in.CapArcStart[i]
		count := in.ArcCount[i]
		for k := range count {
			a := in.CapV[base+k]
			bIndex := base + k + 1
			if k == count-1 {
				bIndex = in.CapWallStart[i]
			}
			b := in.CapV[bIndex]
			if in.Start {
				mesh.AddTriangle([3]int{a, b, apex}, face)
			} else {
				mesh.AddTriangle([3]int{apex, b, a}, face)
			}
		}
		mesh.Bump(face, proofbound.AbsSumUpper(in.ArcSag[i], in.Delta, in.LevelDelta, in.Axial))
		mesh.AddAreaSlack(g.AreaAllow, capBlendPatchFacetAllow(mesh.Vertices(), mesh.Triangles(), count, in.ArcSag[i]))
	}

	for i, w := range in.Walks {
		if err := budget.Step(); err != nil {
			return err
		}
		face, g, err := patch(next + i)
		if err != nil {
			return err
		}
		if g.Circular != w.IsCircular() {
			return fmt.Errorf(`%w: patch %d of the chamfer band on loop %d does not state the geometry of the wall it descends from`, decaderr.ErrDegenerate, next+i, in.Loop)
		}
		count := in.Count[i]
		twist := 0.0
		first := len(mesh.Triangles())
		for k := range count {
			s0 := in.SideStart[i] + k
			s1 := in.SideStart[i] + k + 1
			c0 := in.CapWallStart[i] + k
			c1 := in.CapWallStart[i] + k + 1
			if k == count-1 {
				s1 = in.SideStart[(i+1)%n]
				c1 = CapBlendNextCapSample(in.CapArcStart, in.CapWallStart, (i+1)%n)
			}
			lo0, lo1, hi0, hi1 := in.CapV[c0], in.CapV[c1], in.SideV[s0], in.SideV[s1]
			if !in.Start {
				lo0, lo1, hi0, hi1 = in.SideV[s0], in.SideV[s1], in.CapV[c0], in.CapV[c1]
			}
			mesh.AddTriangle([3]int{lo0, lo1, hi1}, face)
			mesh.AddTriangle([3]int{lo0, hi1, hi0}, face)
			if g.Circular && g.Skew > 0 {
				verts := mesh.Vertices()
				twist = math.Max(twist, proofbound.CellTwistOffsetUpper(
					verts[in.SideV[s0]], verts[in.SideV[s1]],
					verts[in.CapV[c0]], verts[in.CapV[c1]]))
			}
		}
		if proofbound.IsNonFinite(g.Skew) {
			return fmt.Errorf(`%w: a chamfer band patch states no bound on how far its two directrix windows differ`, decaderr.ErrUnsupported)
		}
		locus := 0.0
		radiusRound := 0.0
		sagitta := 0.0
		if in.Joins != nil {
			locus = math.Max(in.LocusGap[i], in.LocusGap[(i+1)%n])
		}
		if w.IsCircular() {
			sagitta = math.Max(in.SideSag[i], in.CapSag[i])
			inside := 1.0
			if w.Th1 < w.Th0 {
				inside = -1
			}
			radiusRound = proofarith.AddRoundError(w.Radius, -inside*in.D, in.CapRadius[i])
		}
		patchDelta := proofbound.AbsSumUpper(twist, sagitta,
			proofbound.ProductUpper(g.CapRadius, g.Skew), locus, radiusRound)
		mesh.Bump(face, proofbound.AbsSumUpper(patchDelta, in.Delta, in.LevelDelta, in.Axial))
		mesh.AddAreaSlack(g.AreaAllow, capBlendFacetAllow(mesh.Vertices(), mesh.Triangles(), first, patchDelta))
	}
	return nil
}

func capBlendFacetAllow(verts []r3.Vec, tris [][3]int, first int, delta float64) float64 {
	if delta <= 0 {
		return 0
	}
	total := 0.0
	for _, tri := range tris[first:] {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		total = proofbound.AbsSumUpper(total, proofbound.PerturbedTriangleAreaAllow(a, b, c, delta))
	}
	return total
}

func capBlendPatchFacetAllow(verts []r3.Vec, tris [][3]int, count int, delta float64) float64 {
	if count <= 0 || count > len(tris) {
		return 0
	}
	return capBlendFacetAllow(verts, tris, len(tris)-count, delta)
}
