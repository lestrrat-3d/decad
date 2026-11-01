package tessellation

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// PrismLoop is one recorded boundary loop's already chorded samples and
// proof charges. FaceOf[j] names the wall of sample j's outgoing chord.
type PrismLoop[F comparable] struct {
	Samples                                  []sectionrecord.Point2
	FaceOf                                   []F
	SagOf                                    []float64
	BoundOf                                  []proofbound.WalkEndBound
	MaxSag, WallSlack, CapSlack, SegmentArea float64
	PerimeterUpper                           float64
	Walks                                    int
}

// PrismTopology holds the shared wall and cap indices and their proof inputs.
type PrismTopology[F comparable] struct {
	Points                                          []sectionrecord.Point2
	Bounds                                          []proofbound.WalkEndBound
	LoopIdx                                         [][]int
	LoopSag                                         []float64
	Triangles                                       [][3]int
	Sources                                         []F
	FaceTrim, FaceAxial                             map[F]float64
	ReplayFaceBounds                                []PrismReplayFaceBound[F]
	AreaSlack, SegmentArea, PerimeterUpper, CapTrim float64
	Walks                                           int
}

// PrismReplayFaceBound holds a private rectangular face's proof inputs.
type PrismReplayFaceBound[F comparable] struct {
	Face        F
	Trim, Axial float64
}

// SetReplayFaceBound updates a preallocated private face record.
func (t *PrismTopology[F]) SetReplayFaceBound(face F, trim, axial float64, maxTrim bool) error {
	for i := range t.ReplayFaceBounds {
		if t.ReplayFaceBounds[i].Face == face {
			if maxTrim {
				t.ReplayFaceBounds[i].Trim = math.Max(t.ReplayFaceBounds[i].Trim, trim)
			} else {
				t.ReplayFaceBounds[i].Trim = trim
			}
			t.ReplayFaceBounds[i].Axial = axial
			return nil
		}
	}
	if len(t.ReplayFaceBounds) == cap(t.ReplayFaceBounds) {
		return fmt.Errorf("%w: prism face record capacity exceeded", decaderr.ErrUnsupported)
	}
	t.ReplayFaceBounds = append(t.ReplayFaceBounds, PrismReplayFaceBound[F]{Face: face, Trim: trim, Axial: axial})
	return nil
}

// PrismWalls emits two facets per boundary chord and accumulates the analytic
// proof terms in recorded loop order. Every 2D sample owns adjacent bottom
// and top vertex indices, which the cap later shares.
func PrismWalls[F comparable](loops []PrismLoop[F], sheet bool, axialDelta float64) PrismTopology[F] {
	t := PrismTopology[F]{FaceTrim: map[F]float64{}, FaceAxial: map[F]float64{}}
	for _, loop := range loops {
		t.CapTrim = math.Max(t.CapTrim, loop.MaxSag)
		if sheet {
			t.AreaSlack = proofbound.AbsSumUpper(t.AreaSlack, loop.WallSlack)
		} else {
			t.AreaSlack = proofbound.AbsSumUpper(t.AreaSlack, loop.WallSlack, loop.CapSlack, loop.CapSlack)
		}
		t.SegmentArea = proofbound.AbsSumUpper(t.SegmentArea, loop.SegmentArea)
		t.Walks += loop.Walks
		t.PerimeterUpper = proofbound.AbsSumUpper(t.PerimeterUpper, loop.PerimeterUpper)

		base := len(t.Points)
		t.Points = append(t.Points, loop.Samples...)
		t.Bounds = append(t.Bounds, loop.BoundOf...)
		idx := make([]int, len(loop.Samples))
		for j := range loop.Samples {
			idx[j] = base + j
		}
		t.LoopIdx = append(t.LoopIdx, idx)
		t.LoopSag = append(t.LoopSag, loop.MaxSag)
		for j := range loop.Samples {
			g0 := base + j
			g1 := base + (j+1)%len(loop.Samples)
			f := loop.FaceOf[j]
			t.Triangles = append(t.Triangles,
				[3]int{2 * g0, 2 * g1, 2*g1 + 1},
				[3]int{2 * g0, 2*g1 + 1, 2*g0 + 1})
			t.Sources = append(t.Sources, f, f)
			t.FaceTrim[f] = math.Max(t.FaceTrim[f], loop.SagOf[j])
			t.FaceAxial[f] = axialDelta
		}
	}
	return t
}

// PrismCaps first proves cross-loop clearance, then maps one shared 2D
// triangulation onto both ends. A sheet has no caps and keeps the wall loop's
// minimum sample check.
func PrismCaps[F comparable](ctx context.Context, t *PrismTopology[F], sheet bool,
	capStart, capEnd F, z0Delta, z1Delta float64,
	clearance func(context.Context, []sectionrecord.Point2, [][]int, []float64) error,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error)) error {
	if !sheet {
		if t.ReplayFaceBounds != nil {
			if err := t.SetReplayFaceBound(capStart, t.CapTrim, z0Delta, false); err != nil {
				return err
			}
			if err := t.SetReplayFaceBound(capEnd, t.CapTrim, z1Delta, false); err != nil {
				return err
			}
		} else {
			t.FaceTrim[capStart] = t.CapTrim
			t.FaceTrim[capEnd] = t.CapTrim
			t.FaceAxial[capStart] = z0Delta
			t.FaceAxial[capEnd] = z1Delta
		}
	}
	if err := clearance(ctx, t.Points, t.LoopIdx, t.LoopSag); err != nil {
		return err
	}
	if sheet {
		if len(t.LoopIdx) == 0 || len(t.LoopIdx[0]) < 3 {
			return fmt.Errorf(`%w: a surface-result wall loop needs at least three boundary samples`, decaderr.ErrDegenerate)
		}
		return nil
	}
	capTris, err := triangulate(ctx, t.Points, t.LoopIdx)
	if err != nil {
		return err
	}
	for _, tri := range capTris {
		t.Triangles = append(t.Triangles,
			[3]int{2*tri[0] + 1, 2*tri[1] + 1, 2*tri[2] + 1},
			[3]int{2 * tri[0], 2 * tri[2], 2 * tri[1]})
		t.Sources = append(t.Sources, capEnd, capStart)
	}
	return nil
}

// PrismPatternMeshRoot returns the first section index of the root wall's
// trailing run in a copied prism's paired axial section mesh. The mesh holds
// n bottom/top vertex pairs (verts[2j] at z0, verts[2j+1] at z1), its facets
// number ntri, and source names each facet's wall. roots lists the wall once
// per origin carrying the root role, so a repeated entry is a duplicate root.
// Every refusal is an ErrUnsupported.
func PrismPatternMeshRoot[F comparable](verts []r3.Vec, ntri int, source []F, roots []F, z0, z1 float64) (int, error) {
	if len(verts)%2 != 0 || len(verts) < 8 || ntri < len(verts) || len(source) != ntri {
		return 0, fmt.Errorf("%w: copied prism has no paired axial section mesh", decaderr.ErrUnsupported)
	}
	n := len(verts) / 2
	if len(roots) > 1 {
		return 0, fmt.Errorf("%w: copied prism has duplicate root wall", decaderr.ErrUnsupported)
	}
	if len(roots) == 0 {
		return 0, fmt.Errorf("%w: copied prism lost its root wall", decaderr.ErrUnsupported)
	}
	root := roots[0]
	rootStart := -1
	for j := range n {
		if verts[2*j].Z != z0 || verts[2*j+1].Z != z1 || source[2*j] != source[2*j+1] {
			return 0, fmt.Errorf("%w: copied prism lost its shared axial stations", decaderr.ErrUnsupported)
		}
		if source[2*j] == root {
			if rootStart < 0 {
				rootStart = j
			}
		} else if rootStart >= 0 {
			return 0, fmt.Errorf("%w: copied prism root wall is not one trailing run", decaderr.ErrUnsupported)
		}
	}
	if rootStart < 2 || rootStart >= n-1 {
		return 0, fmt.Errorf("%w: copied prism has no exterior path beside its root wall", decaderr.ErrUnsupported)
	}
	return rootStart, nil
}
