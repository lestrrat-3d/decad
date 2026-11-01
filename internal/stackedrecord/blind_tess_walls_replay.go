package stackedrecord

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindWallSource names the authenticated column and source segment
// whose built face owns a wall triangle.
type ReplayBlindWallSource struct{ Column, Segment int }

// ReplayBlindMeshWalls is the charged vertex and wall-index stage. It does
// not publish a mesh because planar patches and volume proof are pending.
type ReplayBlindMeshWalls struct {
	Vertices  ReplayBlindMeshVertices
	Triangles [][3]int
	Source    []ReplayBlindWallSource
	Faces     []*topo.Face
}

// ReplayBlindMeshWallsWithSource builds wall triangle indices in ordinary
// Tessellate order under the same private consumer meter.
func ReplayBlindMeshWallsWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindMeshWalls, bool, error) {
	vertices, ok, err := ReplayBlindMeshVerticesWithSource(ctx, body, host, requestedChord, budget)
	if err != nil || !ok {
		return ReplayBlindMeshWalls{}, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok || len(vertices.Chords.Columns) != len(vertices.Chords.Loops) ||
		len(vertices.Bottom) != len(vertices.Chords.Loops) || len(vertices.Top) != len(vertices.Chords.Loops) {
		return ReplayBlindMeshWalls{}, false, nil
	}
	var count uint64
	for _, loop := range vertices.Chords.Loops {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindMeshWalls{}, false, err
		}
		count += uint64(len(loop.Samples))
		if count > 1<<18 || len(loop.FaceOf) != len(loop.Samples) {
			return ReplayBlindMeshWalls{}, false, decaderr.ErrUnsupported
		}
	}
	if err := budget.ReserveArray(ctx, 2*count, uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return ReplayBlindMeshWalls{}, false, err
	}
	if err := budget.ReserveArray(ctx, 2*count, uint64(unsafe.Sizeof(ReplayBlindWallSource{}))); err != nil {
		return ReplayBlindMeshWalls{}, false, err
	}
	if err := budget.ReserveArray(ctx, 2*count, uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return ReplayBlindMeshWalls{}, false, err
	}
	out := ReplayBlindMeshWalls{Vertices: vertices,
		Triangles: make([][3]int, 0, 2*count),
		Source:    make([]ReplayBlindWallSource, 0, 2*count),
		Faces:     make([]*topo.Face, 0, 2*count)}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return ReplayBlindMeshWalls{}, false, err
	}
	reflected := sp.OuterPrism().Reflected()
	for ci, loop := range vertices.Chords.Loops {
		if len(vertices.Bottom[ci]) != len(loop.Samples) || len(vertices.Top[ci]) != len(loop.Samples) {
			return ReplayBlindMeshWalls{}, false, decaderr.ErrUnsupported
		}
		for j, segment := range loop.FaceOf {
			if err := budget.SpendWork(ctx, 24); err != nil {
				return ReplayBlindMeshWalls{}, false, err
			}
			if segment < 0 || segment >= len(vertices.Chords.Columns[ci].Loop.Segments) {
				return ReplayBlindMeshWalls{}, false, decaderr.ErrUnsupported
			}
			col := vertices.Chords.Columns[ci]
			if err := budget.ReserveBundle(ctx, 128, 1, 512); err != nil {
				return ReplayBlindMeshWalls{}, false, err
			}
			role := fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
				col.Start, col.Region, col.LoopIndex, segment)
			var face *topo.Face
			for _, lump := range body.Lumps {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return ReplayBlindMeshWalls{}, false, err
				}
				for _, shell := range lump.Shells {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return ReplayBlindMeshWalls{}, false, err
					}
					for _, candidate := range shell.Faces {
						if err := budget.SpendWork(ctx, uint64(len(candidate.Origins))+1); err != nil {
							return ReplayBlindMeshWalls{}, false, err
						}
						if faceHasRole(candidate, role) {
							if face != nil {
								return ReplayBlindMeshWalls{}, false, decaderr.ErrUnsupported
							}
							face = candidate
						}
					}
				}
			}
			if face == nil {
				return ReplayBlindMeshWalls{}, false, decaderr.ErrUnsupported
			}
			next := (j + 1) % len(loop.Samples)
			a := [3]int{vertices.Bottom[ci][j], vertices.Bottom[ci][next], vertices.Top[ci][next]}
			b := [3]int{vertices.Bottom[ci][j], vertices.Top[ci][next], vertices.Top[ci][j]}
			if reflected {
				a[1], a[2] = a[2], a[1]
				b[1], b[2] = b[2], b[1]
			}
			out.Triangles = append(out.Triangles, a, b)
			owner := ReplayBlindWallSource{Column: ci, Segment: segment}
			out.Source = append(out.Source, owner, owner)
			out.Faces = append(out.Faces, face, face)
		}
	}
	return out, true, nil
}
