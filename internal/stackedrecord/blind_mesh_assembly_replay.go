package stackedrecord

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindMeshAssembly is a source-bound but unverified mesh candidate.
// Closedness, face bounds and occupied-volume proof follow this stage.
type ReplayBlindMeshAssembly struct {
	Mesh      *topo.Mesh
	Rings     []tessellation.StackedRing[*topo.Face]
	Store     []float64
	StoreMax  float64
	FaceTrim  map[*topo.Face]float64
	FaceAxial map[*topo.Face]float64
}

func replayBlindFaceByRole(ctx context.Context, body *topo.Body, role string,
	budget *proofbound.ReplayBudget) (*topo.Face, error) {
	if body == nil || role == "" || len(role) > 256 {
		return nil, decaderr.ErrUnsupported
	}
	var found *topo.Face
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, uint64(len(face.Origins))+1); err != nil {
					return nil, err
				}
				if faceHasRole(face, role) {
					if found != nil {
						return nil, decaderr.ErrUnsupported
					}
					found = face
				}
			}
		}
	}
	if found == nil {
		return nil, decaderr.ErrUnsupported
	}
	return found, nil
}

func replayBlindPatchLimit(ctx context.Context, loops []PatchLoop,
	chords ReplayBlindChordStructure, budget *proofbound.ReplayBudget) (uint64, error) {
	if len(loops) == 0 || len(loops) > 4096 {
		return 0, decaderr.ErrUnsupported
	}
	var samples uint64
	for _, loop := range loops {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		if loop.Column < 0 || loop.Column >= len(chords.Loops) {
			return 0, decaderr.ErrUnsupported
		}
		samples += uint64(len(chords.Loops[loop.Column].Samples))
		if samples > 4096 {
			return 0, decaderr.ErrUnsupported
		}
	}
	if samples < 3 {
		return 0, decaderr.ErrUnsupported
	}
	return samples + 2*uint64(len(loops)-1) - 2, nil
}

// ReplayBlindMeshAssemblyWithSource carries one consumer meter through the
// sealed source walks, wall lifts, face assignment, and every planar patch.
func ReplayBlindMeshAssemblyWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindMeshAssembly, bool, error) {
	walls, ok, err := ReplayBlindMeshWallsWithSource(ctx, body, host, requestedChord, budget)
	if err != nil || !ok {
		return ReplayBlindMeshAssembly{}, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok || len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 || sp.IsGroup() ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return ReplayBlindMeshAssembly{}, false, nil
	}
	chords := walls.Vertices.Chords
	if len(chords.Rows) != 2 {
		return ReplayBlindMeshAssembly{}, false, nil
	}
	patchLimit := uint64(0)
	for _, row := range [...][]SlabLoop{chords.Rows[0], chords.Rows[1]} {
		if err := budget.ReserveArray(ctx, uint64(len(row)), uint64(unsafe.Sizeof(PatchLoop{}))); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		loops := make([]PatchLoop, len(row))
		for i, entry := range row {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			loops[i] = PatchLoop{Column: entry.Column, Outer: entry.Loop == 0}
		}
		limit, err := replayBlindPatchLimit(ctx, loops, chords, budget)
		if err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		patchLimit += limit
	}
	for _, patch := range chords.Patches {
		limit, err := replayBlindPatchLimit(ctx, patch.Loops, chords, budget)
		if err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		patchLimit += limit
	}
	if patchLimit > 1<<18 || uint64(len(walls.Triangles))+patchLimit > 1<<18 {
		return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
	}
	capacity := uint64(len(walls.Triangles)) + patchLimit
	if err := budget.ReserveArray(ctx, capacity, uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveArray(ctx, capacity, uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	faceCount := 0
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			faceCount += len(shell.Faces)
		}
	}
	if faceCount == 0 || faceCount > 4096 {
		return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
	}
	if err := budget.ReserveBundle(ctx, uint64(faceCount)*64+256,
		uint64(faceCount)*2+1, uint64(faceCount)*512+1024); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(chords.Loops)),
		uint64(unsafe.Sizeof(tessellation.StackedRing[*topo.Face]{}))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	mesh := &topo.Mesh{Vertices: walls.Vertices.Vertices,
		Triangles: make([][3]int, 0, capacity), Source: make([]*topo.Face, 0, capacity)}
	mesh.Triangles = append(mesh.Triangles, walls.Triangles...)
	mesh.Source = append(mesh.Source, walls.Faces...)
	out := ReplayBlindMeshAssembly{Mesh: mesh, Store: walls.Vertices.Store,
		Rings:     make([]tessellation.StackedRing[*topo.Face], len(chords.Loops)),
		FaceTrim:  make(map[*topo.Face]float64, faceCount),
		FaceAxial: make(map[*topo.Face]float64, faceCount)}
	wallOffset := 0
	for ci, loop := range chords.Loops {
		if err := budget.ReserveArray(ctx, uint64(len(loop.Samples)),
			uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		faces := make([]*topo.Face, len(loop.Samples))
		col := chords.Columns[ci]
		if col.Start < 0 || col.End >= len(sp.Slabs) {
			return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
		}
		axial := math.Max(sp.Slabs[col.Start].Z0Delta, sp.Slabs[col.End].Z1Delta)
		for j := range loop.Samples {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			face := walls.Faces[2*(wallOffset+j)]
			faces[j] = face
			out.FaceTrim[face] = math.Max(out.FaceTrim[face], loop.SagOf[j])
			out.FaceAxial[face] = axial
		}
		wallOffset += len(loop.Samples)
		out.Rings[ci] = tessellation.StackedRing[*topo.Face]{Samples: loop.Samples,
			Bottom: walls.Vertices.Bottom[ci], Top: walls.Vertices.Top[ci], Faces: faces,
			Sag: loop.MaxSag, SegmentArea: loop.SegmentArea,
			Walks: loop.Walks, PerimeterUpper: loop.PerimeterUpper}
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, loop.WallSlack, loop.CapSlack, loop.CapSlack)
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	reflected := sp.OuterPrism().Reflected()
	emit := func(role string, source []PatchLoop, reverse bool, axial float64) error {
		face, err := replayBlindFaceByRole(ctx, body, role, budget)
		if err != nil {
			return err
		}
		if err := budget.ReserveArray(ctx, uint64(len(source)),
			uint64(unsafe.Sizeof(tessellation.StackedPatchLoop{}))); err != nil {
			return err
		}
		loops := make([]tessellation.StackedPatchLoop, len(source))
		for i, entry := range source {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if entry.Column < 0 || entry.Column >= len(chords.Columns) {
				return decaderr.ErrUnsupported
			}
			loops[i] = tessellation.StackedPatchLoop{Column: entry.Column,
				Top: entry.Top, Outer: entry.Outer,
				ColumnOuter: chords.Columns[entry.Column].LoopIndex == 0}
		}
		patch, err := ReplayBlindPatchTriangles(ctx, out.Rings, loops, reverse, budget)
		if err != nil {
			return err
		}
		for _, tri := range patch.Triangles {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return err
			}
			if reflected {
				tri[1], tri[2] = tri[2], tri[1]
			}
			mesh.Triangles = append(mesh.Triangles, tri)
			mesh.Source = append(mesh.Source, face)
		}
		out.FaceTrim[face] = math.Max(out.FaceTrim[face], patch.Sag)
		out.FaceAxial[face] = axial
		return nil
	}
	for slab, role := range [...]string{provenance.RoleCapStart, provenance.RoleCapEnd} {
		row := chords.Rows[slab]
		if err := budget.ReserveArray(ctx, uint64(len(row)), uint64(unsafe.Sizeof(PatchLoop{}))); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		loops := make([]PatchLoop, len(row))
		for i, entry := range row {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			if entry.Region != 0 {
				return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
			}
			loops[i] = PatchLoop{Column: entry.Column, Top: slab == 1, Outer: entry.Loop == 0}
		}
		axial := sp.Slabs[0].Z0Delta
		if slab == 1 {
			axial = sp.Slabs[1].Z1Delta
		}
		if err := emit(role, loops, slab == 0, axial); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
	}
	for _, patch := range chords.Patches {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		if err := emit(patch.Role, patch.Loops, !patch.Floor, sp.Slabs[0].Z1Delta); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
	}
	if len(mesh.Triangles) != len(mesh.Source) {
		return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
	}
	out.StoreMax, err = tessellation.ReplayStoreMax(ctx, out.Store, budget)
	if err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	return out, true, nil
}
