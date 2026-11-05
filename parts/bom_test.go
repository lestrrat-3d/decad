package parts_test

import (
	"bytes"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func bomSet(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	s := parts.NewSet(doc)
	petg := parts.Material{Name: "PETG", Density: units.GramsPerCubicCentimeter(1.27)}
	mustAdd(t, s,
		parts.Part{Name: "zeta", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 20, 20, 0, 10), Material: petg},
		parts.Part{Name: "pin-b", Kind: parts.Manufactured, Body: cylinder(t, doc, 30, 3, 0, 10), Material: petg,
			Spec: "pin 6 x 10"},
		parts.Part{Name: "pin-a", Kind: parts.Manufactured, Body: cylinder(t, doc, 40, 3, 0, 10), Material: petg,
			Spec: "pin 6 x 10"},
		parts.Part{Name: "band-1", Kind: parts.Purchased, Quantity: 2, Spec: "rubber band"},
		parts.Part{Name: "bolt", Kind: parts.Purchased, Quantity: 4, Material: parts.Material{Name: "steel"}})
	return s
}

func TestBOM(t *testing.T) {
	s := bomSet(t)
	bom, err := s.BOM(t.Context())
	require.NoError(t, err)
	// Manufactured rows first, then purchased; each by spec.
	var specs []string
	for _, r := range bom.Rows {
		specs = append(specs, r.Spec)
	}
	require.Equal(t, []string{"pin 6 x 10", "zeta", "bolt", "rubber band"}, specs)

	pin, plate, bolt, band := bom.Rows[0], bom.Rows[1], bom.Rows[2], bom.Rows[3]
	require.Equal(t, []string{"pin-b", "pin-a"}, pin.Parts)
	require.Equal(t, 2, pin.Quantity)
	require.InDelta(t, math.Pi*9*10, pin.Volume.Value.Base(), 0.5)
	require.InDelta(t, 1, plate.Quantity, 0)
	require.InDelta(t, 4000, plate.Volume.Value.Base(), 1e-6)
	// 4000 mm³ is 4 cm³ of PETG at 1.27 g/cm³.
	grams, err := plate.Mass.Value.In(units.Gram)
	require.NoError(t, err)
	require.InDelta(t, 5.08, grams, 1e-9)
	require.Equal(t, plate.Volume.Exactness, plate.Mass.Exactness)
	require.Nil(t, bolt.Volume, "a body-less part has no volume")
	require.Nil(t, bolt.Mass)
	require.Equal(t, 4, bolt.Quantity)
	require.Equal(t, "steel", bolt.Material.Name)
	require.Equal(t, 2, band.Quantity)
	require.Nil(t, band.Mass, "no density, no mass")
}

func TestBOMFalsifiesAClaimOfSameness(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "pin-short", Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, 0, 10), Spec: "pin"},
		parts.Part{Name: "pin-long", Kind: parts.Manufactured, Body: cylinder(t, doc, 30, 3, 0, 20), Spec: "pin"})
	_, err := s.BOM(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "pin-short")
	require.ErrorContains(t, err, "pin-long")

	// Different kinds cannot share a row either.
	s2 := parts.NewSet(doc)
	mustAdd(t, s2,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 5, 5, 0, 5), Spec: "x"},
		parts.Part{Name: "b", Kind: parts.Purchased, Quantity: 1, Spec: "x"})
	_, err = s2.BOM(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A density of the wrong kind is refused when the part is added.
	s3 := parts.NewSet(doc)
	err = s3.Add(parts.Part{Name: "c", Kind: parts.Purchased, Quantity: 1,
		Material: parts.Material{Name: "m", Density: units.Millimeters(1)}})
	require.ErrorIs(t, err, decad.ErrUnitKind)
	err = s3.Add(parts.Part{Name: "c", Kind: parts.Purchased, Quantity: 1,
		Material: parts.Material{Name: "m", Density: units.GramsPerCubicCentimeter(-1)}})
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)

	_, err = (*parts.Set)(nil).BOM(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestSummaryMarkdown(t *testing.T) {
	s := bomSet(t)
	var out bytes.Buffer
	require.NoError(t, parts.Summary{Set: s}.WriteMarkdown(&out))
	page := out.String()
	require.Contains(t, page, "| pin 6 x 10 | manufactured | 2 | PETG |")
	require.Contains(t, page, "| zeta | manufactured | 1 | PETG | 4000.00 mm³ | 5.08 g | zeta |")
	require.Contains(t, page, "| rubber band | purchased | 2 | - | - | - | band-1 |")
	require.NotContains(t, page, "## Joints")
	require.NotContains(t, page, "## Exploded view")
	require.Less(t, strings.Index(page, "## Bill of materials"), strings.Index(page, "## Connection diagram"))
	require.Contains(t, page, "```mermaid\nflowchart LR\n")
	require.True(t, strings.HasSuffix(page, "```\n"))
}

func TestSummaryMarkdownSections(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlate(t, doc)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, -2, 17)})
	mustJoin(t, s, parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate, Motion: parts.Turns})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	rep.Unexplained = []parts.PairFinding{{A: "x", B: "y", Near: &decad.Measurement{Value: units.Millimeters(0.05),
		Exactness: decad.Approximate, Bound: units.Millimeters(0.05)}}}

	var out bytes.Buffer
	require.NoError(t, parts.Summary{Set: s, Joints: rep, Exploded: ex}.WriteMarkdown(&out))
	page := out.String()
	order := []string{"## Bill of materials", "## Joints", "## Unexplained pairs", "## Exploded view", "## Connection diagram"}
	last := -1
	for _, h := range order {
		i := strings.Index(page, h)
		require.Greater(t, i, last, h)
		last = i
	}
	require.Contains(t, page, "| Joint | Kind | A | B | Motion | Status | Cause | Evidence |")
	require.Contains(t, page, "| pin~plate | pin in bore | pin | plate | turns | confirmed | - | measured 0.20")
	require.Contains(t, page, "engaged 10.00 mm; axis from faces |")
	require.Contains(t, page, "## Unexplained pairs")
	require.Contains(t, page, "| x | y |")
	require.Contains(t, page, "| within 0.10 mm |")
	require.Contains(t, page, "Ground: plate.")
	require.Contains(t, page, "| pin | 1 | axis from faces | - | (0.00, 0.00, 22.00) | 22.00 | 0.00 | clear |")
	require.Contains(t, page, `pin -- "pin in bore, turns, confirmed" --> plate`)

	require.ErrorIs(t, parts.Summary{}.WriteMarkdown(&out), decad.ErrDegenerate)
}
