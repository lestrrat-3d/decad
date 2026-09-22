package decad

import (
	"fmt"

	"github.com/lestrrat-go/option/v3"
)

// This file is the shared surface-result vocabulary of docs/surface-design.md
// §3-§4: the one option every wall-building feature accepts,
// WithSurfaceResult, and the two refusal helpers that keep a sheet body out
// of an operation Table X or Table R does not admit it to. Extrude, Revolve
// and Loft each wire the option into their own build (extrude.go,
// revolve_build.go, loft_build.go); Sweep refuses it outright (sweep.go);
// boolean.go, fillet.go, chamfer.go, shell.go and stops.go consume
// refuseSheetOperand at their own gates.
//
// It also holds the two topology helpers every surface-result build shares
// (docs/surface-design.md §2.2): shellIsOpen, which reads a shell's open
// state off its faces' own edge adjacency rather than the option that built
// it, and splitConnectedFaces/sheetLumps, which give a face set the Lump
// each of its truly connected pieces is entitled to instead of one lump
// regardless of connectivity.

// SurfaceResultOption configures every feature WithSurfaceResult reaches
// (docs/surface-design.md §3). A feature this evaluator cannot yet build as a
// surface refuses it with [ErrUnsupported] (Table R row R1).
type SurfaceResultOption interface {
	ExtrudeOption
	RevolveOption
	SweepOption
	LoftOption
}

type surfaceResultOption struct{ option.Interface }

func (surfaceResultOption) extrudeOption() {}
func (surfaceResultOption) revolveOption() {}
func (surfaceResultOption) sweepOption()   {}
func (surfaceResultOption) loftOption()    {}

type identSurfaceResult struct{}

// WithSurfaceResult builds the feature's wall set and omits every face that
// exists only to close the solid, publishing a sheet body — Kind() ==
// BodySheet — instead (docs/surface-design.md §4.1). The option carries no
// payload of its own; its identity is the whole of the signal, and a repeated
// WithSurfaceResult() is idempotent, never an error.
func WithSurfaceResult() SurfaceResultOption {
	return surfaceResultOption{option.New(identSurfaceResult{}, struct{}{})}
}

// refuseSurfaceResult reports [ErrUnsupported] when o is a
// WithSurfaceResult() option, for a feature Table R row R1 stages as a
// permanent or not-yet-built refusal. It asserts on the concrete type alone
// and never calls Ident(): the payload is never read, so the type itself is
// the whole check.
func refuseSurfaceResult(o any, feature string) error {
	if _, ok := o.(surfaceResultOption); ok {
		return fmt.Errorf(`%w: %s does not build a surface result (docs/surface-design.md Table R row R1)`, ErrUnsupported, feature)
	}
	return nil
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
	parent := make(map[*Face]*Face, len(faces))
	for _, f := range faces {
		parent[f] = f
	}
	byEdge := map[*Edge][]*Face{}
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				byEdge[ce.edge] = append(byEdge[ce.edge], f)
			}
		}
	}
	for _, group := range byEdge {
		for i := 1; i < len(group); i++ {
			ra, rb := unionFindRoot(parent, group[0]), unionFindRoot(parent, group[i])
			if ra != rb {
				parent[ra] = rb
			}
		}
	}
	order := make([]*Face, 0, len(faces))
	groups := map[*Face][]*Face{}
	for _, f := range faces {
		root := unionFindRoot(parent, f)
		if _, ok := groups[root]; !ok {
			order = append(order, root)
		}
		groups[root] = append(groups[root], f)
	}
	out := make([][]*Face, len(order))
	for i, root := range order {
		out[i] = groups[root]
	}
	return out
}

// unionFindRoot is splitConnectedFaces's own path-compressing find: it walks
// parent to f's representative, flattening every visited link to that
// representative's grandparent on the way so a later find over the same set
// stays cheap. A top-level function rather than a closure over parent, since
// a self-referential closure cannot be declared and assigned in one
// statement (staticcheck S1021 does not account for the recursion).
func unionFindRoot(parent map[*Face]*Face, f *Face) *Face {
	for parent[f] != f {
		parent[f] = parent[parent[f]]
		f = parent[f]
	}
	return f
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
