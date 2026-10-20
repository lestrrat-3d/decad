// Package topobuild holds the topology assembly steps several feature builders
// share, over internal/topo nodes: registering faces on their edges, splitting
// a face set into lumps and shells, the node walkers a builder reads its own
// body back through, re-minting blend roles, and the last finiteness gate on a
// built body's measurements (agent-docs/evaluator-design.md §3, §5;
// agent-docs/surface-design.md §2.2). It allocates every node through the caller's
// topo.Alloc.
package topobuild

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegroup"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// AttachFaceLoops links each coedge's shared edge to its face. The
// assembly is private until the caller commits the body, but it can be large
// for a recorded region with many loops, so every nesting level polls ctx.
func AttachFaceLoops(ctx context.Context, faces []*topo.Face) error {
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, l := range f.Loops {
			if err := ctx.Err(); err != nil {
				return err
			}
			for _, ce := range l.CoEdges {
				if err := ctx.Err(); err != nil {
					return err
				}
				ce.Edge.Faces = append(ce.Edge.Faces, f)
			}
		}
	}
	return nil
}

// ShellIsOpen reports whether faces holds at least one free edge: an edge
// with exactly one adjacent face (agent-docs/surface-design.md §2.2). It is
// derived from an edge that actually exists and never from the option that
// built the shell, so it reads the same on a hand-built fixture as it does
// on a feature's own output. Every caller MUST run it AFTER
// AttachFaceLoops has registered each face on its edges: called
// earlier, every edge reports zero adjacent faces and this reports every
// shell open, including a solid's.
func ShellIsOpen(faces []*topo.Face) bool {
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				if len(ce.Edge.Faces) == 1 {
					return true
				}
			}
		}
	}
	return false
}

// SplitConnectedFaces partitions faces into its connected pieces, two faces
// joined whenever a loop of one shares an Edge with a loop of the other
// (agent-docs/surface-design.md §2.2: a lump is a connected piece of a body, not a
// connected SOLID piece — a sheet's wall groups that touch nowhere, once
// their closing caps are omitted, are separate pieces even though a solid
// built from the same record is one). Each returned slice keeps faces' own
// order; the slices themselves are ordered by the first face of theirs that
// appears in faces.
func SplitConnectedFaces(faces []*topo.Face) [][]*topo.Face {
	byEdge := map[*topo.Edge][]*topo.Face{}
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				byEdge[ce.Edge] = append(byEdge[ce.Edge], f)
			}
		}
	}
	return surfacegroup.Connected(faces, byEdge)
}

// SheetLumps builds one Lump per connected piece of faces, each holding one
// FaceShell whose Open is ShellIsOpen's own reading and whose Void stays
// false — a sheet bounds no cavity (agent-docs/surface-design.md §2.2), so nothing
// here plays the void role a solid's own nested shell does. It must run
// AFTER every face in faces has been registered on its edges
// (ShellIsOpen's own contract): the prism and revolve surface-result builds
// both call it once their walls are fully assembled, never before. Each
// node comes from alloc, and no faces gives no lumps (nil).
func SheetLumps(alloc topo.Alloc, faces []*topo.Face) []*topo.Lump {
	groups := SplitConnectedFaces(faces)
	if len(groups) == 0 {
		return nil
	}
	lumps := make([]*topo.Lump, len(groups))
	for i, g := range groups {
		shell := alloc.NewFaceShell()
		shell.Open = ShellIsOpen(g)
		shell.Faces = g
		lump := alloc.NewLump()
		lump.Shells = []*topo.FaceShell{shell}
		lumps[i] = lump
	}
	return lumps
}

// AddBlendRoles gives every blend wall its second kind(i,j) role (Table B): the
// wall built from a blend connector already carries side(i,j); the second role —
// "fillet" for Fillet, "chamfer" for Chamfer — names the same (loop, segment) of
// the rewritten record.
func AddBlendRoles(ctx context.Context, body *topo.Body, ref provenance.Producer, blendSegs []map[int]struct{}, kind string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	for li, segs := range blendSegs {
		if err := ctx.Err(); err != nil {
			return err
		}
		for sj := range segs {
			if err := ctx.Err(); err != nil {
				return err
			}
			side := fmt.Sprintf("side(%d,%d)", li, sj)
			blend := fmt.Sprintf("%s(%d,%d)", kind, li, sj)
			for _, f := range Faces(body) {
				if err := ctx.Err(); err != nil {
					return err
				}
				for _, o := range f.Origins {
					if err := ctx.Err(); err != nil {
						return err
					}
					if o.Producer == ref && o.Role == side {
						f.Origins = append(f.Origins, provenance.Ref{Producer: ref, Role: blend})
						break
					}
				}
			}
		}
	}
	return nil
}
