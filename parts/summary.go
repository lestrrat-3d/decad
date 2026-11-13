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
// joint table with each joint's status, cause and evidence, and the unexplained
// pairs, when Joints is set; the exploded depth
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
		b.WriteString("\n## Joints\n\n| Joint | Kind | A | B | Motion | Status | Cause | Evidence |\n|---|---|---|---|---|---|---|---|\n")
		result := map[string]JointResult{}
		for _, jr := range s.Joints.Joints {
			result[jr.Joint.Name] = jr
		}
		for _, j := range s.Set.joints {
			jr := result[j.Name]
			cause := "-"
			if jr.Cause != CauseNone {
				cause = jr.Cause.String()
			}
			fmt.Fprintf(&b, "| %s | %s | %s | %s | %s | %s | %s | %s |\n", cell(j.Name), j.Kind, cell(j.A), cell(j.B), j.Motion,
				jr.Status, cause, cell(evidence(jr)))
		}
		if len(s.Joints.Unexplained) > 0 {
			b.WriteString("\n## Unexplained pairs\n\nBoxes meet and no joint names the pair; this is where to look, not a finding.\n\n")
			b.WriteString("| A | B | Overlap min (mm) | Overlap max (mm) | Near |\n|---|---|---|---|---|\n")
			for _, p := range s.Joints.Unexplained {
				o := p.Overlap
				near := "-"
				if p.Near != nil {
					near = "within " + measure(&decad.Measurement{Value: units.Millimeters(p.Near.Value.Base() + p.Near.Bound.Base()),
						Exactness: decad.Exact}, units.Millimeter, "mm")
				}
				fmt.Fprintf(&b, "| %s | %s | (%.2f, %.2f, %.2f) | (%.2f, %.2f, %.2f) | %s |\n", cell(p.A), cell(p.B), o.Min.X,
					o.Min.Y, o.Min.Z, o.Max.X, o.Max.Y, o.Max.Z, near)
			}
		}
	}
	if s.Exploded != nil {
		fmt.Fprintf(&b, "\n## Exploded view\n\nGround: %s.\n\n", s.Exploded.Ground)
		b.WriteString("| Part | Depth | Pull | Stage | Offset (mm) | Moved (mm) | Clearance (mm) | Path |\n|---|---|---|---|---|---|---|---|\n")
		for _, p := range s.Exploded.Parts {
			path := p.Path.String()
			if p.Blocker != "" && (p.Path == PathBlocked || p.Path == PathEnclosed) {
				path += " by " + cell(p.Blocker)
			}
			fmt.Fprintf(&b, "| %s | %d | %s | %s | (%.2f, %.2f, %.2f) | %.2f | %.2f | %s |\n", cell(p.Name), p.Depth, p.Pull,
				cell(p.Stage), p.Offset.X, p.Offset.Y, p.Offset.Z, p.Moved.Base(), p.Clearance.Base(), path)
		}
		if len(s.Exploded.Short) > 0 {
			b.WriteString("\nParts closer to a part they are not joined to than their gap:")
			for _, n := range s.Exploded.Short {
				fmt.Fprintf(&b, " %s;", n)
			}
			b.WriteString("\n")
		}
		if len(s.Exploded.Overlaps) > 0 {
			b.WriteString("\nExploded boxes that still meet:")
			for _, o := range s.Exploded.Overlaps {
				fmt.Fprintf(&b, " %s and %s;", o[0], o[1])
			}
			b.WriteString("\n")
		}
		if sp := s.Exploded.Spread; sp != nil && sp.Ran {
			fmt.Fprintf(&b, "\nSpread: %d layers along (%g, %g, %g), stepped along (%g, %g, %g), %.4g x %.4g x %.4g mm; "+
				"seen along (%g, %g, %g) no part wholly hides a part it is joined to.\n",
				sp.Layers, sp.Axis.X, sp.Axis.Y, sp.Axis.Z, sp.Shear.X, sp.Shear.Y, sp.Shear.Z,
				sp.Width.Base(), sp.Depth.Base(), sp.Height.Base(), sp.View.X, sp.View.Y, sp.View.Z)
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

// evidence is the numbers a joint's status rests on, for the joint table.
func evidence(r JointResult) string {
	var parts []string
	if r.Measured != nil {
		parts = append(parts, "measured "+measure(r.Measured, units.Millimeter, "mm"))
	}
	if r.Engaged != nil {
		parts = append(parts, "engaged "+measure(r.Engaged, units.Millimeter, "mm"))
	}
	if r.Axis != nil {
		parts = append(parts, "axis from "+r.AxisFrom.String())
	}
	if r.Reason != "" {
		parts = append(parts, r.Reason)
	}
	return strings.Join(parts, "; ")
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
