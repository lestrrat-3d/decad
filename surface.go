package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/surfacegroup"
)

// This file is the shared surface-result vocabulary of docs/surface-design.md
// §3-§4: the one option every wall-building feature accepts,
// WithSurfaceResult, and the refusal helper that keeps a sheet body out of an
// operation Table X does not admit it to. Extrude, Revolve, Sweep and Loft
// each wire the option into their own build (extrude.go, revolve_build.go,
// sweep.go/sweep_arc.go/sweep_composite.go, loft_build.go) — Sweep is the
// last of the four to take it, so Table R row R1 now names no feature this
// evaluator still refuses outright; the row stays in the design as the stated
// rule for one added later. boolean.go, fillet.go, chamfer.go, shell.go and
// stops.go consume refuseSheetOperand at their own gates.
//
// Its topology adapters read face and edge adjacency for internal/surfacegroup:
// shellIsOpen reads a shell's open state from its edges, while
// splitConnectedFaces/sheetLumps build one Lump per connected face group.

// SurfaceResultOption configures every feature WithSurfaceResult reaches
// (docs/surface-design.md §3). A feature this evaluator cannot yet build as a
// surface refuses it with [ErrUnsupported] (Table R row R1).
type SurfaceResultOption interface {
	ExtrudeOption
	RevolveOption
	SweepOption
	LoftOption
}

// WithSurfaceResult builds the feature's wall set and omits every face that
// exists only to close the solid, publishing a sheet body — Kind() ==
// BodySheet — instead (docs/surface-design.md §4.1). The option carries no
// payload of its own; its identity is the whole of the signal, and a repeated
// WithSurfaceResult() is idempotent, never an error.
func WithSurfaceResult() SurfaceResultOption {
	return surfaceResultOptionValue{featureoption.WithSurfaceResult()}
}

// refuseSheetOperand reports [ErrUnsupported] when b is live and a sheet
// (Kind() == BodySheet), for an operation docs/surface-design.md Table X does
// not admit a sheet to. A nil b names nothing to refuse.
func refuseSheetOperand(b *Body, operation string) error {
	if b != nil && b.Kind() == BodySheet {
		return fmt.Errorf(`%w: %s does not accept a sheet body (docs/surface-design.md Table X)`, ErrUnsupported, operation)
	}
	return nil
}

// shellIsOpen reports whether faces holds at least one free edge: an edge
// with exactly one adjacent face (docs/surface-design.md §2.2). It is
// derived from an edge that actually exists and never from the option that
// built the shell, so it reads the same on a hand-built fixture as it does
// on a feature's own output. Every caller MUST run it AFTER
// attachFaceLoopsContext has registered each face on its edges: called
// earlier, every edge reports zero adjacent faces and this reports every
// shell open, including a solid's.
func shellIsOpen(faces []*Face) bool {
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				if len(ce.edge.faces) == 1 {
					return true
				}
			}
		}
	}
	return false
}

// splitConnectedFaces partitions faces into its connected pieces, two faces
// joined whenever a loop of one shares an Edge with a loop of the other
// (docs/surface-design.md §2.2: a lump is a connected piece of a body, not a
// connected SOLID piece — a sheet's wall groups that touch nowhere, once
// their closing caps are omitted, are separate pieces even though a solid
// built from the same record is one). Each returned slice keeps faces' own
// order; the slices themselves are ordered by the first face of theirs that
// appears in faces.
func splitConnectedFaces(faces []*Face) [][]*Face {
	byEdge := map[*Edge][]*Face{}
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				byEdge[ce.edge] = append(byEdge[ce.edge], f)
			}
		}
	}
	return surfacegroup.Connected(faces, byEdge)
}

// freeChainCountsByFace groups the body's own recorded free Edges by their
// single adjacent face and counts, per face, the CONNECTED COMPONENTS its
// free Edges form over their own Vertex pointers — one boundary CHAIN per
// component, not one per Edge object. A coalesced wall's rim is usually one
// recorded curve, a single segment or several merged collinear ones, so it
// is usually one Edge object AND one chain. But two of a face's free Edges
// that share an interned Vertex — a revolve wall's two rims meeting at a pole
// the payload interns between them — are still only ONE chain: the two
// Edges are two chords of the same connected boundary walk, exactly as
// internal/tessellation.RequireSheetBoundary reads the mesh side of the same
// face by connected component rather than by directed-edge count.
func freeChainCountsByFace(b *Body) map[*Face]int {
	byFace := map[*Face][][2]*Vertex{}
	for _, e := range b.Edges() {
		if !e.IsFree() {
			continue
		}
		faces := e.Faces()
		if len(faces) != 1 {
			continue // IsFree already guarantees exactly one; defensive only
		}
		byFace[faces[0]] = append(byFace[faces[0]], [2]*Vertex{e.Start(), e.End()})
	}
	counts := make(map[*Face]int, len(byFace))
	for face, edges := range byFace {
		counts[face] = surfacegroup.ChainCount(edges)
	}
	return counts
}

// sheetLumps builds one Lump per connected piece of faces, each holding one
// Shell whose IsOpen is shellIsOpen's own reading and whose IsVoid stays
// false — a sheet bounds no cavity (docs/surface-design.md §2.2), so nothing
// here plays the void role a solid's own nested shell does. It must run
// AFTER every face in faces has been registered on its edges
// (shellIsOpen's own contract): the prism and revolve surface-result builds
// both call it once their walls are fully assembled, never before.
func sheetLumps(faces []*Face) []*Lump {
	groups := splitConnectedFaces(faces)
	lumps := make([]*Lump, len(groups))
	for i, g := range groups {
		lumps[i] = &Lump{shells: []*Shell{{faces: g, open: shellIsOpen(g)}}}
	}
	return lumps
}
