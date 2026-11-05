package parts

import (
	"fmt"
	"io"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// Summary is the agent-readable text form of the outputs. Every field but Set
// is optional. It has no extension hook: a caller who wants its own sections on
// the page writes them to the same io.Writer after WriteMarkdown.
type Summary struct {
	Set      *Set
	Joints   *JointReport
	Exploded *Exploded
}

// WriteMarkdown writes one page, in this order: the bill of materials table; the
// joint table with each joint's status when Joints is set; the exploded depth
// and pull table when Exploded is set; the Mermaid connection diagram. Numbers
// print with their bounds. A Set with a retired body is decad.ErrRetiredBody
// and a nil Set decad.ErrDegenerate.
func (s Summary) WriteMarkdown(w io.Writer) error {
	if s.Set == nil || s.Set.doc == nil || w == nil {
		return fmt.Errorf("parts: %w: nil set or writer", decad.ErrDegenerate)
	}
	bom, err := s.Set.bom()
	if err != nil {
		return err
	}
	var b strings.Builder
	b.WriteString("# Build summary\n\n## Bill of materials\n\n")
	b.WriteString("| Spec | Kind | Quantity | Material | Volume | Mass | Parts |\n|---|---|---|---|---|---|---|\n")
	for _, r := range bom.Rows {
		fmt.Fprintf(&b, "| %s | %s | %d | %s | %s | %s | %s |\n", cell(r.Spec), r.Kind, r.Quantity, cell(r.Material.Name),
			measure(r.Volume, units.CubicMillimeter, "mm³"), measure(r.Mass, units.Gram, "g"),
			cell(strings.Join(r.Parts, ", ")))
	}
	b.WriteString("\nVolume is that of one part. Mass is the solid body's mass: infill lowers it.\n")
	if s.Joints != nil {
		b.WriteString("\n## Joints\n\n| Joint | Kind | A | B | Motion | Status |\n|---|---|---|---|---|---|\n")
		status := map[string]JointStatus{}
		for _, jr := range s.Joints.Joints {
			status[jr.Joint.Name] = jr.Status
		}
		for _, j := range s.Set.joints {
			fmt.Fprintf(&b, "| %s | %s | %s | %s | %s | %s |\n", cell(j.Name), j.Kind, cell(j.A), cell(j.B), j.Motion,
				status[j.Name])
		}
	}
	if s.Exploded != nil {
		fmt.Fprintf(&b, "\n## Exploded view\n\nGround: %s.\n\n", s.Exploded.Ground)
		b.WriteString("| Part | Depth | Pull | Offset (mm) |\n|---|---|---|---|\n")
		for _, p := range s.Exploded.Parts {
			fmt.Fprintf(&b, "| %s | %d | %s | (%.2f, %.2f, %.2f) |\n", cell(p.Name), p.Depth, p.Pull, p.Offset.X, p.Offset.Y,
				p.Offset.Z)
		}
		if len(s.Exploded.Overlaps) > 0 {
			b.WriteString("\nExploded boxes that still meet:")
			for _, o := range s.Exploded.Overlaps {
				fmt.Fprintf(&b, " %s and %s;", o[0], o[1])
			}
			b.WriteString("\n")
		}
	}
	b.WriteString("\n## Connection diagram\n\n```mermaid\n")
	if err := s.Set.Graph(s.Joints).WriteMermaid(&b); err != nil {
		return err
	}
	b.WriteString("```\n")
	_, err = io.WriteString(w, b.String())
	return err
}

// cell makes text safe inside a table cell.
func cell(s string) string {
	if s == "" {
		return "-"
	}
	return strings.NewReplacer("|", "\\|", "\n", " ").Replace(s)
}

// measure prints a reading in the unit u with its bound when it has one, or "-"
// for none.
func measure(m *decad.Measurement, u units.Unit, symbol string) string {
	if m == nil {
		return "-"
	}
	v, err := m.Value.In(u)
	if err != nil {
		return "-"
	}
	if m.Exactness == decad.Exact {
		return fmt.Sprintf("%.2f %s", v, symbol)
	}
	b, err := m.Bound.In(u)
	if err != nil {
		return fmt.Sprintf("%.2f %s", v, symbol)
	}
	return fmt.Sprintf("%.2f ± %.2g %s", v, b, symbol)
}
