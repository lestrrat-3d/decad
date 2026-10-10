package sketchrecord

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/sketch"
)

// AdmitProfile runs the public seam gates before matching a fresh snapshot.
// Recording reads that fresh match instead of caller-mutable p.
func AdmitProfile(s *sketch.Sketch, p *sketch.Profile) (*sketch.Profile, error) {
	if s == nil || p == nil {
		return nil, fmt.Errorf(`%w: RecordProfile requires a sketch and a profile`, decaderr.ErrDegenerate)
	}
	if p.Sketch() != s {
		return nil, fmt.Errorf(`%w: the profile was built from a different sketch, so its plane-local coordinates are another plane's`, decaderr.ErrForeignProfile)
	}
	if p.IsStale() {
		return nil, fmt.Errorf(`%w: the sketch has changed since this profile was built; rebuild with Sketch.Profiles or Sketch.UnionProfiles`, decaderr.ErrStaleProfile)
	}
	if !p.Valid {
		return nil, fmt.Errorf(`%w: a self-intersecting or degenerate region is never silently swept`, decaderr.ErrInvalidProfile)
	}
	return AuthenticateProfile(s, p)
}

// AuthenticateProfile checks boundary ownership and exact snapshot content.
// Its caller has already checked source, freshness, and validity.
func AuthenticateProfile(s *sketch.Sketch, p *sketch.Profile) (*sketch.Profile, error) {
	// Boundary ownership is checked before reading fresh geometry, so a
	// caller-supplied foreign or typed-nil entity never reaches Geometry.
	owned := make(map[sketch.Entity]struct{})
	for _, ent := range s.Entities() {
		owned[ent] = struct{}{}
	}
	if err := authenticateBoundaryLoop(owned, p.Outer); err != nil {
		return nil, err
	}
	for _, hole := range p.Holes {
		if err := authenticateBoundaryLoop(owned, hole); err != nil {
			return nil, err
		}
	}
	// A union's private member indices identify its source regions. Rebuild from
	// Sketch and compare the entire snapshot; the caller's boundary is not input.
	if members := p.UnionRegionIndices(); len(members) != 0 {
		trusted, err := s.UnionProfiles(members...)
		if err != nil || trusted == nil || !sameProfileSnapshot(p, trusted) {
			return nil, fmt.Errorf(`%w: the union profile no longer matches the sketch's current selected regions; rebuild with Sketch.UnionProfiles`, decaderr.ErrInvalidProfile)
		}
		return trusted, nil
	}

	var trusted *sketch.Profile
	for _, candidate := range s.Profiles() {
		if !sameProfileSnapshot(p, candidate) {
			continue
		}
		if trusted != nil {
			return nil, fmt.Errorf(`%w: the profile snapshot matches more than one current region; rebuild with Sketch.Profiles`, decaderr.ErrInvalidProfile)
		}
		trusted = candidate
	}
	if trusted == nil {
		return nil, fmt.Errorf(`%w: the profile snapshot was altered after Sketch.Profiles returned it; rebuild and pass the profile unchanged`, decaderr.ErrInvalidProfile)
	}
	return trusted, nil
}

func authenticateBoundaryLoop(owned map[sketch.Entity]struct{}, edges []sketch.BoundaryEdge) error {
	for _, edge := range edges {
		if isNilSketchEntity(edge.Entity) {
			return fmt.Errorf(`%w: the profile boundary contains a nil entity; rebuild with Sketch.Profiles`, decaderr.ErrInvalidProfile)
		}
		if _, ok := owned[edge.Entity]; !ok {
			return fmt.Errorf(`%w: the profile boundary contains an entity not owned by its source sketch`, decaderr.ErrForeignProfile)
		}
	}
	return nil
}

// isNilSketchEntity checks the sealed sketch entity pointer variants without
// reflection. A typed nil is an invalid boundary before any geometry read.
func isNilSketchEntity(ent sketch.Entity) bool {
	switch ent := ent.(type) {
	case nil:
		return true
	case *sketch.Line:
		return ent == nil
	case *sketch.Circle:
		return ent == nil
	case *sketch.Arc:
		return ent == nil
	case *sketch.Ellipse:
		return ent == nil
	case *sketch.EllipticalArc:
		return ent == nil
	case *sketch.Conic:
		return ent == nil
	case *sketch.Spline:
		return ent == nil
	case *sketch.ClosedSpline:
		return ent == nil
	case *sketch.FitSpline:
		return ent == nil
	case *sketch.NURBS:
		return ent == nil
	default:
		return false
	}
}

func sameProfileSnapshot(a, b *sketch.Profile) bool {
	if a.Sketch() != b.Sketch() || a.Revision() != b.Revision() ||
		a.Area != b.Area || a.Valid != b.Valid || a.SelfIntersecting != b.SelfIntersecting ||
		!slices.Equal(a.UnionRegionIndices(), b.UnionRegionIndices()) ||
		(a.Entities == nil) != (b.Entities == nil) || !slices.Equal(a.Entities, b.Entities) ||
		(a.Holes == nil) != (b.Holes == nil) || !sameBoundaryLoop(a.Outer, b.Outer) ||
		len(a.Holes) != len(b.Holes) {
		return false
	}
	for i := range a.Holes {
		if !sameBoundaryLoop(a.Holes[i], b.Holes[i]) {
			return false
		}
	}
	return true
}

func sameBoundaryLoop(a, b []sketch.BoundaryEdge) bool {
	if (a == nil) != (b == nil) || len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameBoundaryEdge(a[i], b[i]) {
			return false
		}
	}
	return true
}

func sameBoundaryEdge(a, b sketch.BoundaryEdge) bool {
	return a.Entity == b.Entity &&
		a.Partial == b.Partial &&
		a.Reversed == b.Reversed &&
		a.TStart == b.TStart &&
		a.TEnd == b.TEnd &&
		a.TExact == b.TExact &&
		(a.Polyline == nil) == (b.Polyline == nil) &&
		slices.Equal(a.Polyline, b.Polyline)
}

// admitChain runs the public seam gates before matching a fresh chain.
func admitChain(s *sketch.Sketch, ch *sketch.Chain) (*sketch.Chain, error) {
	if s == nil || ch == nil {
		return nil, fmt.Errorf(`%w: RecordChain requires a sketch and a chain`, decaderr.ErrDegenerate)
	}
	if ch.Sketch() != s {
		return nil, fmt.Errorf(`%w: the chain was built from a different sketch, so its plane-local coordinates are another plane's`, decaderr.ErrForeignProfile)
	}
	if ch.IsStale() {
		return nil, fmt.Errorf(`%w: the sketch has changed since this chain was built; rebuild with Sketch.Chains`, decaderr.ErrStaleProfile)
	}
	if !ch.Valid {
		return nil, fmt.Errorf(`%w: a self-intersecting or degenerate walk is never silently swept`, decaderr.ErrInvalidProfile)
	}
	return AuthenticateChain(s, ch)
}

// AuthenticateChain checks ownership and exact snapshot content. It also
// authenticates invalid chains for callers that need to inspect the match
// before applying the public validity gate. Nil and foreign chain edges use
// the chain's ErrForeignProfile refusal class.
func AuthenticateChain(s *sketch.Sketch, ch *sketch.Chain) (*sketch.Chain, error) {
	owned := make(map[sketch.Entity]struct{})
	for _, ent := range s.Entities() {
		owned[ent] = struct{}{}
	}
	if err := authenticateChainEdges(owned, ch.Edges); err != nil {
		return nil, err
	}

	var trusted *sketch.Chain
	for _, candidate := range s.Chains() {
		if !SameChainSnapshot(ch, candidate) {
			continue
		}
		if trusted != nil {
			return nil, fmt.Errorf(`%w: the chain snapshot matches more than one current chain; rebuild with Sketch.Chains`, decaderr.ErrInvalidProfile)
		}
		trusted = candidate
	}
	if trusted == nil {
		return nil, fmt.Errorf(`%w: the chain snapshot was altered after Sketch.Chains returned it; rebuild and pass the chain unchanged`, decaderr.ErrInvalidProfile)
	}
	return trusted, nil
}

func authenticateChainEdges(owned map[sketch.Entity]struct{}, edges []sketch.BoundaryEdge) error {
	for _, edge := range edges {
		if isNilSketchEntity(edge.Entity) {
			return fmt.Errorf(`%w: the chain contains a nil entity; rebuild with Sketch.Chains`, decaderr.ErrForeignProfile)
		}
		if _, ok := owned[edge.Entity]; !ok {
			return fmt.Errorf(`%w: the chain contains an entity not owned by its source sketch`, decaderr.ErrForeignProfile)
		}
	}
	return nil
}

// SameChainSnapshot compares every exported Chain field that the caller can
// change: entities, edges, length, validity, and self-intersection status.
// The edge comparison also distinguishes nil slices from empty slices.
func SameChainSnapshot(a, b *sketch.Chain) bool {
	return a.Sketch() == b.Sketch() && a.Revision() == b.Revision() &&
		a.Length == b.Length && a.Valid == b.Valid && a.SelfIntersecting == b.SelfIntersecting &&
		(a.Entities == nil) == (b.Entities == nil) && slices.Equal(a.Entities, b.Entities) &&
		sameBoundaryLoop(a.Edges, b.Edges)
}
