package stackedrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Column is one wall loop that continues through adjacent slabs.
type Column struct {
	Loop              sectionrecord.LoopRecord
	Start, End        int
	Region, LoopIndex int
}

// SlabLoop identifies one region loop and its wall column in one slab.
type SlabLoop struct {
	Column, Region, Loop int
}

// Columns groups equal loop records into uninterrupted wall columns.
func Columns(sp Record) ([]Column, [][]SlabLoop, error) {
	var columns []Column
	bySlab := make([][]SlabLoop, len(sp.Slabs))
	for k, slab := range sp.Slabs {
		for r, region := range slab.Regions {
			for i, loop := range append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...) {
				found := -1
				if k != 0 {
					for _, prev := range bySlab[k-1] {
						if equalLoop(loop, columns[prev.Column].Loop) {
							found = prev.Column
							break
						}
					}
				}
				switch {
				case found < 0:
					found = len(columns)
					columns = append(columns, Column{Loop: loop, Start: k, End: k, Region: r, LoopIndex: i})
				case columns[found].End == k:
					return nil, nil, fmt.Errorf(`%w: slab %d carries one loop record twice`, decaderr.ErrDegenerate, k)
				default:
					columns[found].End = k
				}
				bySlab[k] = append(bySlab[k], SlabLoop{Column: found, Region: r, Loop: i})
			}
		}
	}
	return columns, bySlab, nil
}

// PatchLoop names a wall column ring bounding an exposed interface patch.
type PatchLoop struct {
	Column int
	Top    bool
	Outer  bool
}

// Patch is one exposed planar patch and the wall column rings bounding it.
type Patch struct {
	Role   string
	Floor  bool
	Record momentinput.Profile
	Loops  []PatchLoop
}

// InterfacePatches pairs each exposed patch with the wall rings that bound it.
func InterfacePatches(sp Record, columns []Column, bySlab [][]SlabLoop, k int) ([]Patch, error) {
	boundary := sp.Interfaces[k]
	if wideLower, lining := LiningSides(sp.Slabs[k], sp.Slabs[k+1]); lining {
		return liningPatches(sp, columns, bySlab, k, wideLower)
	}
	lower, upper := sp.Slabs[k].Regions[0], sp.Slabs[k+1].Regions[0]
	var patches []Patch
	if !equalLoop(lower.Outer, upper.Outer) {
		lowerOuter, upperOuter := slabColumn(bySlab[k], 0, 0), slabColumn(bySlab[k+1], 0, 0)
		if lowerOuter < 0 || upperOuter < 0 || columns[lowerOuter].End != k || columns[upperOuter].Start != k+1 {
			return nil, fmt.Errorf(`%w: interface %d changes the outer loop but a wall column crosses it`, decaderr.ErrDegenerate, k)
		}
		for e, exposed := range boundary.LowerExposed {
			patches = append(patches, Patch{Role: fmt.Sprintf("floor(%d,%d)", k, e), Floor: true, Record: exposed,
				Loops: []PatchLoop{{Column: lowerOuter, Top: true, Outer: true}, {Column: upperOuter}}})
		}
		for e, exposed := range boundary.UpperExposed {
			patches = append(patches, Patch{Role: fmt.Sprintf("ceiling(%d,%d)", k, e), Record: exposed,
				Loops: []PatchLoop{{Column: upperOuter, Outer: true}, {Column: lowerOuter, Top: true}}})
		}
		return patches, nil
	}
	lowerOnly, upperOnly := ExclusiveHoles(lower, upper)
	if len(lowerOnly) != 0 && len(upperOnly) != 0 {
		return enclosingPatch(sp, columns, bySlab, k, lowerOnly, upperOnly)
	}
	for e, hole := range upperOnly {
		ci, err := holeColumn(columns, holeColumns(bySlab[k+1]), hole, k+1, true)
		if err != nil {
			return nil, err
		}
		patches = append(patches, Patch{Role: fmt.Sprintf("floor(%d,%d)", k, e), Floor: true,
			Record: boundary.LowerExposed[e], Loops: []PatchLoop{{Column: ci, Outer: true}}})
	}
	for e, hole := range lowerOnly {
		ci, err := holeColumn(columns, holeColumns(bySlab[k]), hole, k, false)
		if err != nil {
			return nil, err
		}
		patches = append(patches, Patch{Role: fmt.Sprintf("ceiling(%d,%d)", k, e),
			Record: boundary.UpperExposed[e], Loops: []PatchLoop{{Column: ci, Top: true, Outer: true}}})
	}
	return patches, nil
}

// enclosingPatch pairs an annular shoulder with the wall rings on both sides
// of the interface. The outer ring belongs to the newly enlarged hole.
func enclosingPatch(sp Record, columns []Column, bySlab [][]SlabLoop, k int,
	lowerOnly, upperOnly []sectionrecord.LoopRecord) ([]Patch, error) {
	boundary := sp.Interfaces[k]
	newHole, enclosed := upperOnly, lowerOnly
	newSlab, oldSlab, floor := k+1, k, true
	exposed, role := boundary.LowerExposed, fmt.Sprintf("floor(%d,0)", k)
	if len(boundary.UpperExposed) != 0 {
		newHole, enclosed = lowerOnly, upperOnly
		newSlab, oldSlab, floor = k, k+1, false
		exposed, role = boundary.UpperExposed, fmt.Sprintf("ceiling(%d,0)", k)
	}
	if len(newHole) != 1 || len(exposed) != 1 {
		return nil, fmt.Errorf(`%w: enclosing interface %d has no single exposed patch`, decaderr.ErrDegenerate, k)
	}
	newStarts := floor
	outer, err := holeColumn(columns, holeColumns(bySlab[newSlab]), newHole[0], newSlab, newStarts)
	if err != nil {
		return nil, err
	}
	patch := Patch{Role: role, Floor: floor, Record: exposed[0],
		Loops: []PatchLoop{{Column: outer, Top: !floor, Outer: true}}}
	for _, hole := range enclosed {
		oldStarts := !floor
		inner, err := holeColumn(columns, holeColumns(bySlab[oldSlab]), hole, oldSlab, oldStarts)
		if err != nil {
			return nil, err
		}
		patch.Loops = append(patch.Loops, PatchLoop{Column: inner, Top: floor})
	}
	return []Patch{patch}, nil
}

func holeColumns(entries []SlabLoop) []int {
	var out []int
	for _, e := range entries {
		if e.Region == 0 && e.Loop != 0 {
			out = append(out, e.Column)
		}
	}
	return out
}

func holeColumn(columns []Column, candidates []int, hole sectionrecord.LoopRecord, slab int, starts bool) (int, error) {
	for _, ci := range candidates {
		col := columns[ci]
		if equalLoop(col.Loop, hole) && ((starts && col.Start == slab) || (!starts && col.End == slab)) {
			return ci, nil
		}
	}
	return 0, fmt.Errorf(`%w: an exposed patch has no wall column`, decaderr.ErrDegenerate)
}

func slabColumn(entries []SlabLoop, region, loop int) int {
	for _, e := range entries {
		if e.Region == region && e.Loop == loop {
			return e.Column
		}
	}
	return -1
}

func liningPatches(sp Record, columns []Column, bySlab [][]SlabLoop, k int, wideLower bool) ([]Patch, error) {
	narrowSlab, top := k, true
	exposed, role := sp.Interfaces[k].UpperExposed, fmt.Sprintf("ceiling(%d,0)", k)
	if wideLower {
		narrowSlab, top = k+1, false
		exposed, role = sp.Interfaces[k].LowerExposed, fmt.Sprintf("floor(%d,0)", k)
	}
	if len(exposed) != 1 {
		return nil, fmt.Errorf(`%w: lining interface %d records no single exposed patch`, decaderr.ErrDegenerate, k)
	}
	ring := func(region, loop int, outer bool) (PatchLoop, error) {
		ci := slabColumn(bySlab[narrowSlab], region, loop)
		if ci < 0 || (top && columns[ci].End != k) || (!top && columns[ci].Start != k+1) {
			return PatchLoop{}, fmt.Errorf(`%w: lining interface %d has no wall column ending at it`, decaderr.ErrDegenerate, k)
		}
		return PatchLoop{Column: ci, Top: top, Outer: outer}, nil
	}
	outer, err := ring(0, 1, true)
	if err != nil {
		return nil, err
	}
	patch := Patch{Role: role, Floor: wideLower, Record: exposed[0], Loops: []PatchLoop{outer}}
	for m := 1; m < len(sp.Slabs[narrowSlab].Regions); m++ {
		hole, err := ring(m, 0, false)
		if err != nil {
			return nil, err
		}
		patch.Loops = append(patch.Loops, hole)
	}
	return []Patch{patch}, nil
}
