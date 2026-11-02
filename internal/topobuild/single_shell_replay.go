package topobuild

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayRequireSingleClosedShell authenticates the structural single-shell
// case and binds each mesh source to that shell. It does not prove winding.
func ReplayRequireSingleClosedShell(ctx context.Context, m *topo.Mesh, lumps []*topo.Lump, budget *proofbound.ReplayBudget) error {
	if m == nil || budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if len(m.Triangles) != len(m.Source) {
		return decaderr.ErrUnsupported
	}
	var only *topo.FaceShell
	for i := range lumps {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		lump := lumps[i]
		if lump == nil {
			return decaderr.ErrUnsupported
		}
		for j := range lump.Shells {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return err
			}
			shell := lump.Shells[j]
			if shell == nil || shell.Open || only != nil {
				return decaderr.ErrUnsupported
			}
			only = shell
		}
	}
	if only == nil || only.Void || len(only.Faces) == 0 || len(m.Triangles) == 0 {
		return decaderr.ErrUnsupported
	}
	for i := range only.Faces {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		face := only.Faces[i]
		if face == nil {
			return decaderr.ErrUnsupported
		}
		for j := range i {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return err
			}
			if only.Faces[j] == face {
				return decaderr.ErrUnsupported
			}
		}
	}
	for i := range m.Source {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		face := m.Source[i]
		found := false
		for j := range only.Faces {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return err
			}
			if only.Faces[j] == face {
				found = true
				break
			}
		}
		if !found {
			return decaderr.ErrUnsupported
		}
	}
	for i := range only.Faces {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		face := only.Faces[i]
		found := false
		for j := range m.Source {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return err
			}
			if m.Source[j] == face {
				found = true
				break
			}
		}
		if !found {
			return decaderr.ErrUnsupported
		}
	}
	return ctx.Err()
}

// ReplayRequirePositiveSingleShell checks the single-shell source association
// and exact positive signed volume. Closure, vertex links and facet noncontact
// remain separate requirements.
func ReplayRequirePositiveSingleShell(ctx context.Context, m *topo.Mesh, lumps []*topo.Lump, budget *proofbound.ReplayBudget) error {
	if err := ReplayRequireSingleClosedShell(ctx, m, lumps, budget); err != nil {
		return err
	}
	sign, err := tessellation.ReplayOrientationSign(ctx, m.Vertices, m.Triangles, budget)
	if err != nil {
		return err
	}
	if sign <= 0 {
		return decaderr.ErrDegenerate
	}
	return nil
}
