package parts

import (
	"fmt"
	"io"
	"strings"
)

// JointStatus is what Check found for one joint. The zero value says it was not
// evaluated.
type JointStatus int

const (
	NotEvaluated JointStatus = iota
	Confirmed
	Contradicted
	Undecided
	NotChecked
)

// String returns the lower-case word for the status.
func (j JointStatus) String() string {
	switch j {
	case Confirmed:
		return "confirmed"
	case Contradicted:
		return "contradicted"
	case Undecided:
		return "undecided"
	case NotChecked:
		return "not checked"
	}
	return "not evaluated"
}

// Graph is the connection diagram: parts as nodes, joints as typed edges.
type Graph struct {
	Nodes []Node // in Set order
	Edges []Edge // in Join order
}

// Node is one part of the diagram.
type Node struct {
	Part, Group string
	Kind        Kind
}

// Edge is one joint of the diagram.
type Edge struct {
	Joint  string
	A, B   string
	Kind   JointKind
	Motion Motion
	Status JointStatus // NotEvaluated when Graph was given no report
	Cause  Cause       // why the joint is not Confirmed; CauseNone otherwise
}

// Graph returns the connection diagram of the Set. r may be nil; with one, each
// edge carries its joint's status. A part whose body a later modelling step
// retired has an empty Group.
func (s *Set) Graph(r *JointReport) Graph {
	var g Graph
	for _, p := range s.parts {
		grp, _ := s.Group(p.Name)
		g.Nodes = append(g.Nodes, Node{Part: p.Name, Group: grp, Kind: p.Kind})
	}
	result := map[string]JointResult{}
	if r != nil {
		for _, jr := range r.Joints {
			result[jr.Joint.Name] = jr
		}
	}
	for _, j := range s.joints {
		g.Edges = append(g.Edges, Edge{Joint: j.Name, A: j.A, B: j.B, Kind: j.Kind, Motion: j.Motion,
			Status: result[j.Name].Status, Cause: result[j.Name].Cause})
	}
	return g
}

// WriteMermaid writes a left-to-right Mermaid flowchart: one subgraph per
// non-root group, a rectangle for a manufactured part and a rounded node for a
// purchased one, and one edge per joint from A to B labelled with its kind, its
// motion when not fixed, its status when known and, when the joint is not
// Confirmed, its cause in brackets. A Contradicted edge ends in a cross (--x) and
// an Undecided one is dotted; the others keep the plain arrow.
func (g Graph) WriteMermaid(w io.Writer) error {
	ids := map[string]string{}
	used := map[string]struct{}{}
	for _, n := range g.Nodes {
		id := mermaidID(n.Part)
		for k := 2; ; k++ {
			if _, dup := used[id]; !dup {
				break
			}
			id = fmt.Sprintf("%s_%d", mermaidID(n.Part), k)
		}
		used[id] = struct{}{}
		ids[n.Part] = id
	}
	var b strings.Builder
	b.WriteString("flowchart LR\n")
	node := func(indent string, n Node) {
		open, shut := "[", "]"
		if n.Kind == Purchased {
			open, shut = "(", ")"
		}
		fmt.Fprintf(&b, "%s%s%s\"%s\"%s\n", indent, ids[n.Part], open, mermaidText(n.Part), shut)
	}
	var groups []string
	byGroup := map[string][]Node{}
	for _, n := range g.Nodes {
		if n.Group == "" {
			node("  ", n)
			continue
		}
		if _, ok := byGroup[n.Group]; !ok {
			groups = append(groups, n.Group)
		}
		byGroup[n.Group] = append(byGroup[n.Group], n)
	}
	for i, grp := range groups {
		fmt.Fprintf(&b, "  subgraph g%d[\"%s\"]\n", i, mermaidText(grp))
		for _, n := range byGroup[grp] {
			node("    ", n)
		}
		b.WriteString("  end\n")
	}
	for _, e := range g.Edges {
		label := e.Kind.String()
		if e.Motion != Fixed {
			label += ", " + e.Motion.String()
		}
		if e.Status != NotEvaluated {
			label += ", " + e.Status.String()
			if e.Cause != CauseNone {
				label += " (" + e.Cause.String() + ")"
			}
		}
		switch e.Status {
		case Contradicted:
			fmt.Fprintf(&b, "  %s -- \"%s\" --x %s\n", ids[e.A], mermaidText(label), ids[e.B])
		case Undecided:
			fmt.Fprintf(&b, "  %s -. \"%s\" .-> %s\n", ids[e.A], mermaidText(label), ids[e.B])
		default:
			fmt.Fprintf(&b, "  %s -- \"%s\" --> %s\n", ids[e.A], mermaidText(label), ids[e.B])
		}
	}
	_, err := io.WriteString(w, b.String())
	return err
}

// mermaidID makes a node id from a part name: letters, digits and underscores,
// never the word "end".
func mermaidID(name string) string {
	var b strings.Builder
	for _, r := range name {
		switch {
		case r >= 'a' && r <= 'z', r >= 'A' && r <= 'Z', r >= '0' && r <= '9', r == '_':
			b.WriteRune(r)
		default:
			b.WriteByte('_')
		}
	}
	id := b.String()
	if id == "" || strings.EqualFold(id, "end") || (id[0] >= '0' && id[0] <= '9') {
		id = "p_" + id
	}
	return id
}

func mermaidText(s string) string { return strings.NewReplacer(`"`, "#quot;", "\n", " ").Replace(s) }
