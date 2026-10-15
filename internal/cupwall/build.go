package cupwall

import (
	"context"
	"fmt"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RoleShellCap names the pocket floor of a cup.
const RoleShellCap = "shellCap"

// plan builds a cup's stack under the cup's own roles (Table B, B5/B6):
// outer walls side(i,j) indexing loop i of O, cavity walls shellSide(i,j)
// indexing loop i of C, the kept cap capStart, the pocket floor shellCap and
// the rims rim(i), one per wall band. A column crossing the interface is a
// loop of O; a column of the wall slab alone is a loop of C. The offset
// region's columns — C's inward, O's outward — carry offsetDelta.
type plan struct {
	floor       int
	sense       Sense
	offsetDelta float64
}

// WallRoleLoop is the column's loop index in O, or in C for a cavity column.
// A wall-slab region m >= 1 lines O's hole m-1 with C's hole m-1, so loop m
// of either region; region 0 holds both outers, loop 0 of each.
func (p plan) WallRoleLoop(col stackedrecord.WallColumn) int {
	if col.Start != col.End && col.Start == p.floor {
		return col.LoopIndex
	}
	return col.Region
}

func (p plan) NameWalls(ctx context.Context, col stackedrecord.WallColumn, faces []*topo.Face,
	ref provenance.Producer) error {
	if col.Start != col.End {
		return nil
	}
	return renameCavityRoles(ctx, faces, ref)
}

func (p plan) CapRole(slab, region int, _ bool) string {
	if slab == p.floor {
		return provenance.RoleCapStart
	}
	return fmt.Sprintf("rim(%d)", region)
}

func (plan) PatchRole(stackedrecord.Patch) string { return RoleShellCap }

func (p plan) ColumnDelta(col stackedrecord.WallColumn) float64 {
	if (col.Start == col.End) == (p.sense != Outward) {
		return p.offsetDelta
	}
	return 0
}

// Order lists the faces as the cup always has: outer walls, cavity walls, the
// kept cap, the pocket floor, then the rims, each group in build Order.
func (plan) Order(faces []*topo.Face) []*topo.Face {
	rank := func(f *topo.Face) int {
		role := f.Origins[0].Role
		switch {
		case strings.HasPrefix(role, "side("):
			return 0
		case strings.HasPrefix(role, "shellSide("):
			return 1
		case role == provenance.RoleCapStart:
			return 2
		case role == RoleShellCap:
			return 3
		default:
			return 4
		}
	}
	out := slices.Clone(faces)
	slices.SortStableFunc(out, func(a, b *topo.Face) int { return rank(a) - rank(b) })
	return out
}

// Build builds the cup's stack through the stacked build (§9.1) under the
// cup's plan, and keeps the cup record as the body's payload.
func Build(ctx context.Context, env topo.Env, cp Payload) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body, err := stackedrecord.Build(ctx, env, cp.Stack,
		plan{floor: cp.floorSlab(), sense: cp.Sense, offsetDelta: cp.OffsetDelta})
	if err != nil {
		return nil, err
	}
	body.Payload = cp
	return body, nil
}

// renameCavityRoles rewrites the cavity walls' provenance from the
// prismrecord.BuildLoopSidesAs default side(i,j) to shellSide(i,j) — the role Table B
// (B5/B6) gives a cavity wall, indexing loop i of the cavity region Q in the
// result's own record (§11). The cavity walls are built with roleLoop equal to
// their Q loop index, so the rename keeps the index and only swaps the tag.
func renameCavityRoles(ctx context.Context, faces []*topo.Face, ref provenance.Producer) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for i, o := range f.Origins {
			if err := ctx.Err(); err != nil {
				return err
			}
			if o.Producer != ref {
				continue
			}
			var li, j int
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n == 2 {
				f.Origins[i] = provenance.Ref{Producer: ref, Role: fmt.Sprintf("shellSide(%d,%d)", li, j)}
			}
		}
	}
	return nil
}
