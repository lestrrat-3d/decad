// Package parts is a generic schematic and exploded view of a build made of
// separate bodies (agent-docs/parts-design.md). A Set names the placed bodies
// of one document and the joints between them; Explode, Graph and the other
// queries read the Set and change neither it nor the document.
//
// parts knows nothing of how a part is made.
package parts

import (
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// Kind says how a part is obtained. The zero value is reserved; Add refuses it.
type Kind int

const (
	KindUnset    Kind = iota
	Manufactured      // made for this build, by any process
	Purchased         // bought: a band, a bearing, a screw
)

// String returns the lower-case word for the kind.
func (k Kind) String() string {
	switch k {
	case Manufactured:
		return "manufactured"
	case Purchased:
		return "purchased"
	}
	return "unset"
}

// Material names what a part is made of. Name is a free label ("PETG",
// "steel"). Density is a units.Density, or the zero Value when unknown; the
// bill of materials then omits the mass.
type Material struct {
	Name    string
	Density units.Value
}

// Part is one piece of the build.
type Part struct {
	// Name is unique within its Set and non-empty.
	Name string
	Kind Kind
	// Body is the part as placed in the assembled pose. A Manufactured part
	// MUST have one; a Purchased part may (a modeled band) or may not (a screw
	// that is only listed). Every body of a Set belongs to one document.
	Body     *decad.Body
	Material Material
	// Quantity is how many of a body-less part the build needs. A part with a
	// Body is one piece; its Quantity MUST be 0 or 1.
	Quantity int
	// Spec groups identical parts in the bill of materials ("pin 4 x 60",
	// "rubber band 2 x 60"). Parts sharing a non-empty Spec are one row.
	Spec string
}

// Set is the parts of one build and the joints between them. It is a builder:
// its methods edit the Set, never the document. A Set is not safe for
// concurrent mutation; queries may run concurrently while nothing mutates it.
type Set struct {
	doc    *decad.Document
	parts  []Part
	index  map[string]int
	bodies map[*decad.Body]struct{}
	joints []Joint
	jnames map[string]struct{}
}

// NewSet returns an empty Set over doc. doc MUST NOT be nil.
func NewSet(doc *decad.Document) *Set {
	return &Set{doc: doc, index: map[string]int{}, bodies: map[*decad.Body]struct{}{}, jnames: map[string]struct{}{}}
}

// Document returns the document the Set's bodies belong to.
func (s *Set) Document() *decad.Document { return s.doc }

// Add appends p. It refuses, with decad.ErrDegenerate, an empty or repeated
// name, KindUnset, a Manufactured part without a body, a sheet body, a body
// already in the Set and a Quantity outside its rule. A body of another
// document is decad.ErrForeignBody.
func (s *Set) Add(p Part) error {
	if s == nil || s.doc == nil {
		return fmt.Errorf("parts: %w: nil set or document", decad.ErrDegenerate)
	}
	if p.Name == "" {
		return fmt.Errorf("parts: %w: a part needs a name", decad.ErrDegenerate)
	}
	if _, dup := s.index[p.Name]; dup {
		return fmt.Errorf("parts: %w: part %q is already in the set", decad.ErrDegenerate, p.Name)
	}
	switch p.Kind {
	case Manufactured, Purchased:
	default:
		return fmt.Errorf("parts: %w: part %q has no kind", decad.ErrDegenerate, p.Name)
	}
	if err := checkDensity(p.Material.Density); err != nil {
		return fmt.Errorf("parts: part %q: %w", p.Name, err)
	}
	if p.Body == nil {
		if p.Kind == Manufactured {
			return fmt.Errorf("parts: %w: manufactured part %q has no body", decad.ErrDegenerate, p.Name)
		}
		if p.Quantity < 0 {
			return fmt.Errorf("parts: %w: part %q has a negative quantity", decad.ErrDegenerate, p.Name)
		}
	} else {
		if p.Quantity < 0 || p.Quantity > 1 {
			return fmt.Errorf("parts: %w: part %q has a body, so its quantity is 0 or 1", decad.ErrDegenerate, p.Name)
		}
		if p.Body.Document() != s.doc {
			return fmt.Errorf("parts: part %q: %w", p.Name, decad.ErrForeignBody)
		}
		if !p.Body.IsSolid() {
			return fmt.Errorf("parts: %w: part %q is not a solid", decad.ErrDegenerate, p.Name)
		}
		if _, dup := s.bodies[p.Body]; dup {
			return fmt.Errorf("parts: %w: the body of part %q is already in the set", decad.ErrDegenerate, p.Name)
		}
		s.bodies[p.Body] = struct{}{}
	}
	s.index[p.Name] = len(s.parts)
	s.parts = append(s.parts, p)
	return nil
}

// Parts returns the parts in Add order.
func (s *Set) Parts() []Part { return slices.Clone(s.parts) }

// Part returns the named part, or decad.ErrDegenerate for an unknown name.
func (s *Set) Part(name string) (Part, error) {
	i, ok := s.index[name]
	if !ok {
		return Part{}, fmt.Errorf("parts: %w: unknown part %q", decad.ErrDegenerate, name)
	}
	return s.parts[i], nil
}

// Group returns the container path of the named part's body, joined with "/",
// or "" for the root container and for a body-less part. A body consumed by a
// later modelling step is decad.ErrRetiredBody.
func (s *Set) Group(name string) (string, error) {
	p, err := s.Part(name)
	if err != nil {
		return "", err
	}
	if p.Body == nil {
		return "", nil
	}
	c, err := s.doc.ContainerOf(p.Body)
	if err != nil {
		return "", fmt.Errorf("parts: part %q: %w", name, err)
	}
	var path []string
	for ; c != nil && c.Parent() != nil; c = c.Parent() {
		path = append(path, c.Name())
	}
	slices.Reverse(path)
	return strings.Join(path, "/"), nil
}

// live checks that every body of the Set is still live in its document.
func (s *Set) live() error {
	if s == nil || s.doc == nil {
		return fmt.Errorf("parts: %w: nil set or document", decad.ErrDegenerate)
	}
	for _, p := range s.parts {
		if p.Body == nil {
			continue
		}
		if _, err := s.doc.ContainerOf(p.Body); err != nil {
			return fmt.Errorf("parts: part %q: %w", p.Name, err)
		}
	}
	return nil
}

// checkDensity validates a material density: the zero Value (unknown), or a
// finite non-negative units.Density.
func checkDensity(d units.Value) error {
	if d == (units.Value{}) {
		return nil
	}
	if d.Kind() != units.Density {
		return fmt.Errorf("density: %w", decad.ErrUnitKind)
	}
	b := d.Base()
	if math.IsNaN(b) || math.IsInf(b, 0) {
		return fmt.Errorf("density: %w", decad.ErrNotFinite)
	}
	if b < 0 {
		return fmt.Errorf("density: %w", decad.ErrNegativeMagnitude)
	}
	return nil
}
