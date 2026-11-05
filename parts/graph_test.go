package parts_test

import (
	"bytes"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/stretchr/testify/require"
)

func TestGraphMermaid(t *testing.T) {
	doc := decad.New()
	rig, err := doc.Root().NewChild("rig")
	require.NoError(t, err)
	base := block(t, doc, 0, 0, 20, 20, 0, 5)
	arm := block(t, doc, 0, 0, 20, 20, 5, 5)
	pin := cylinder(t, doc, 10, 3, 0, 10)
	require.NoError(t, doc.MoveBody(arm, rig))
	require.NoError(t, doc.MoveBody(pin, rig))

	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: base},
		parts.Part{Name: "arm", Kind: parts.Manufactured, Body: arm},
		parts.Part{Name: "pin 1", Kind: parts.Manufactured, Body: pin},
		parts.Part{Name: "band", Kind: parts.Purchased, Quantity: 1})
	mustJoin(t, s,
		parts.Joint{Name: "lap", Kind: parts.FaceContact, A: "arm", B: nameBase},
		parts.Joint{Name: "hinge", Kind: parts.PinInBore, A: "pin 1", B: "arm", Motion: parts.Turns},
		parts.Joint{Name: nameStrap, Kind: parts.Tie, A: "band", B: nameBase})

	g := s.Graph(nil)
	require.Len(t, g.Nodes, 4)
	require.Equal(t, "rig", g.Nodes[1].Group)
	require.Equal(t, "", g.Nodes[0].Group)
	require.Equal(t, parts.Purchased, g.Nodes[3].Kind)
	require.Len(t, g.Edges, 3)
	require.Equal(t, parts.NotEvaluated, g.Edges[0].Status)

	var out bytes.Buffer
	require.NoError(t, g.WriteMermaid(&out))
	lines := strings.Split(strings.TrimRight(out.String(), "\n"), "\n")
	require.Equal(t, []string{
		"flowchart LR",
		`  base["base"]`,
		`  band("band")`,
		`  subgraph g0["rig"]`,
		`    arm["arm"]`,
		`    pin_1["pin 1"]`,
		`  end`,
		`  arm -- "face contact" --> base`,
		`  pin_1 -- "pin in bore, turns" --> arm`,
		`  band -- "tie" --> base`,
	}, lines)

	// A report adds each joint's status to its edge.
	r := &parts.JointReport{Joints: []parts.JointResult{
		{Joint: parts.Joint{Name: "hinge"}, Status: parts.Confirmed},
		{Joint: parts.Joint{Name: "lap"}, Status: parts.Contradicted},
	}}
	g = s.Graph(r)
	require.Equal(t, parts.Contradicted, g.Edges[0].Status)
	require.Equal(t, parts.Confirmed, g.Edges[1].Status)
	require.Equal(t, parts.NotEvaluated, g.Edges[2].Status)
	out.Reset()
	require.NoError(t, g.WriteMermaid(&out))
	require.Contains(t, out.String(), `arm -- "face contact, contradicted" --> base`)
	require.Contains(t, out.String(), `pin_1 -- "pin in bore, turns, confirmed" --> arm`)
}

func TestGraphMermaidNames(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "end", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 5, 5, 0, 5)},
		parts.Part{Name: "a-b", Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 15, 5, 0, 5)},
		parts.Part{Name: "a b", Kind: parts.Manufactured, Body: block(t, doc, 20, 0, 25, 5, 0, 5)},
		parts.Part{Name: `say "hi"`, Kind: parts.Manufactured, Body: block(t, doc, 30, 0, 35, 5, 0, 5)})
	var out bytes.Buffer
	require.NoError(t, s.Graph(nil).WriteMermaid(&out))
	require.Equal(t, strings.Join([]string{
		"flowchart LR",
		`  p_end["end"]`,
		`  a_b["a-b"]`,
		`  a_b_2["a b"]`,
		`  say__hi_["say #quot;hi#quot;"]`,
	}, "\n")+"\n", out.String())
}
