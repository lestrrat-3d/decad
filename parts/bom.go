package parts

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// BOM is the bill of materials: manufactured rows first, then purchased ones;
// each by Spec, then name.
type BOM struct{ Rows []BOMRow }

// BOMRow is one line of the bill of materials.
type BOMRow struct {
	Spec     string // or the part name when Spec is empty
	Parts    []string
	Kind     Kind
	Quantity int
	Material Material
	Volume   *decad.Measurement // solid volume of one part; nil for a body-less part
	Mass     *decad.Measurement // Volume times Density; nil without a density
}

// BOM returns the bill of materials. Parts sharing a Spec are one row; they must
// agree: the same Kind and material name, and volumes that overlap within their
// bounds, or BOM returns decad.ErrDegenerate naming the two parts. That check
// only falsifies the caller's claim that the parts are the same. A part with a
// body counts one; a body-less part counts its Quantity. Mass is the solid
// body's mass: infill lowers it.
func (s *Set) BOM(ctx context.Context) (*BOM, error) {
	if ctx == nil || s == nil || s.doc == nil {
		return nil, fmt.Errorf("parts: %w: nil context or set", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return s.bom()
}

func (s *Set) bom() (*BOM, error) {
	if err := s.live(); err != nil {
		return nil, err
	}
	var rows []BOMRow
	at := map[string]int{}
	var volumes []*decad.Measurement // the first body volume of each row, parallel to rows
	var first []string               // the part that gave it
	for _, p := range s.parts {
		key := p.Spec
		if key == "" {
			key = p.Name
		}
		qty := p.Quantity
		var vol *decad.Measurement
		if p.Body != nil {
			qty = 1
			v, err := p.Body.Volume()
			if err != nil {
				return nil, fmt.Errorf("parts: part %q: %w", p.Name, err)
			}
			vol = &v
		}
		i, ok := at[key]
		if !ok {
			at[key] = len(rows)
			rows = append(rows, BOMRow{Spec: key, Parts: []string{p.Name}, Kind: p.Kind, Quantity: qty,
				Material: p.Material, Volume: vol})
			volumes, first = append(volumes, vol), append(first, p.Name)
			continue
		}
		row := &rows[i]
		if row.Kind != p.Kind || row.Material.Name != p.Material.Name {
			return nil, fmt.Errorf("parts: %w: parts %q and %q share spec %q but not their kind and material",
				decad.ErrDegenerate, first[i], p.Name, key)
		}
		if vol != nil && volumes[i] != nil {
			d := math.Abs(vol.Value.Base() - volumes[i].Value.Base())
			if d > vol.Bound.Base()+volumes[i].Bound.Base() {
				return nil, fmt.Errorf("parts: %w: parts %q and %q share spec %q but their volumes differ", decad.ErrDegenerate,
					first[i], p.Name, key)
			}
		}
		if volumes[i] == nil && vol != nil {
			volumes[i], first[i], row.Volume = vol, p.Name, vol
		}
		row.Parts = append(row.Parts, p.Name)
		row.Quantity += qty
	}
	for i := range rows {
		if rows[i].Volume == nil || rows[i].Material.Density == (units.Value{}) {
			continue
		}
		mass, err := massOf(*rows[i].Volume, rows[i].Material.Density)
		if err != nil {
			return nil, fmt.Errorf("parts: spec %q: %w", rows[i].Spec, err)
		}
		rows[i].Mass = &mass
	}
	slices.SortStableFunc(rows, func(a, b BOMRow) int {
		if a.Kind != b.Kind {
			if a.Kind == Manufactured {
				return -1
			}
			return 1
		}
		return cmp.Compare(a.Spec, b.Spec)
	})
	return &BOM{Rows: rows}, nil
}

// massOf multiplies a volume reading by a density, carrying the bound.
func massOf(v decad.Measurement, density units.Value) (decad.Measurement, error) {
	m, err := v.Value.Mul(density)
	if err != nil {
		return decad.Measurement{}, err
	}
	b, err := v.Bound.Mul(density)
	if err != nil {
		return decad.Measurement{}, err
	}
	return decad.Measurement{Value: m, Exactness: v.Exactness, Bound: b}, nil
}
