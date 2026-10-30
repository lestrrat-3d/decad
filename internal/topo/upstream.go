package topo

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/provenance"
)

// AllOrigins is a copy of f's own Origins followed by its Upstream roles, nil
// when it has neither: every role a provenance selector matches the face by.
func (f *Face) AllOrigins() []provenance.Ref {
	if len(f.Upstream) == 0 {
		return slices.Clone(f.Origins)
	}
	out := make([]provenance.Ref, 0, len(f.Origins)+len(f.Upstream))
	out = append(out, f.Origins...)
	return append(out, f.Upstream...)
}

// HasOrigin reports whether ref is one of f's own Origins or Upstream roles.
func (f *Face) HasOrigin(ref provenance.Ref) bool {
	return slices.Contains(f.Origins, ref) || slices.Contains(f.Upstream, ref)
}

// UpstreamRoles is the upstream-origin record a payload carries
// (agent-docs/prism-boolean-design.md §11). Each entry names one face of the
// body by the roles its own producer mints, in order, and lists the operand
// roles that face inherits. It is keyed by role name alone, never by
// producer, so a Transform or Copy that rebuilds the payload under a fresh
// producer re-attaches it to the same faces. It is immutable once a payload
// holds it.
type UpstreamRoles []UpstreamEntry

// UpstreamEntry is one face's row of UpstreamRoles.
type UpstreamEntry struct {
	// Roles are the face's own role names, in Origins order.
	Roles []string
	// Origins are the inherited operand roles.
	Origins []provenance.Ref
}

// Apply sets Upstream on every face of b whose own role names equal an
// entry's Roles, and clears it on every other face. A nil table clears
// every face.
func (t UpstreamRoles) Apply(b *Body) {
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			for _, f := range s.Faces {
				f.Upstream = t.lookup(f)
			}
		}
	}
}

func (t UpstreamRoles) lookup(f *Face) []provenance.Ref {
	for _, e := range t {
		if len(e.Roles) != len(f.Origins) {
			continue
		}
		match := true
		for i, o := range f.Origins {
			if o.Role != e.Roles[i] {
				match = false
				break
			}
		}
		if match {
			return slices.Clone(e.Origins)
		}
	}
	return nil
}
