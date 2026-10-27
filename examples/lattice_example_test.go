package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// GyroidLattice fills a box with a gyroid, the smooth triply periodic wall used
// for light, strong print infill, as a closed mesh body. Level sets the wall's
// thickness and the box's faces cap it, so the mesh is watertight and its volume
// reads. Here a 20 mm cube holds a lattice repeating every 10 mm.
func Example_gyroid_lattice() {
	lattice, err := decad.GyroidLattice(context.Background(), decad.LatticeSpec{
		Min:        r3.Vec{},
		Max:        r3.Vec{X: 20, Y: 20, Z: 20},
		Cell:       units.Millimeters(10),
		Level:      units.Scalar(0.8),
		Resolution: 16,
	})
	if err != nil {
		fmt.Printf("failed to build the lattice: %s\n", err)
		return
	}
	volume, err := lattice.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fraction := volume.Value.Base() / 8000
	fmt.Printf("closed: %t, solid fraction: %.2f\n", lattice.Report().Closed, fraction)
	// Output:
	// closed: true, solid fraction: 0.53
}

// SchwarzPLattice and DiamondLattice take the same spec for the other two common
// triply periodic walls: the primitive lattice of cubic channels and the diamond
// lattice of the carbon crystal. Each field has its own range, so Level runs up
// to 3 for Schwarz P and to √2 for the diamond.
func Example_schwarz_diamond_lattice() {
	spec := decad.LatticeSpec{
		Min:        r3.Vec{},
		Max:        r3.Vec{X: 20, Y: 20, Z: 20},
		Cell:       units.Millimeters(10),
		Resolution: 16,
	}
	for _, kind := range []struct {
		name  string
		build func(context.Context, decad.LatticeSpec) (*decad.MeshBody, error)
		level float64
	}{
		{"Schwarz P", decad.SchwarzPLattice, 1},
		{"diamond", decad.DiamondLattice, 0.5},
	} {
		spec.Level = units.Scalar(kind.level)
		lattice, err := kind.build(context.Background(), spec)
		if err != nil {
			fmt.Printf("failed to build the %s lattice: %s\n", kind.name, err)
			return
		}
		volume, err := lattice.Volume()
		if err != nil {
			fmt.Printf("failed to measure volume: %s\n", err)
			return
		}
		fmt.Printf("%s: closed: %t, solid fraction: %.2f\n", kind.name, lattice.Report().Closed, volume.Value.Base()/8000)
	}
	// Output:
	// Schwarz P: closed: true, solid fraction: 0.57
	// diamond: closed: true, solid fraction: 0.41
}

// NeoviusLattice and IWPLattice add two more triply periodic walls: the Neovius
// surface, whose field runs from -13 to 13, and the IWP surface, whose field runs
// from -5 to 3 so that Level runs up to 5.
func Example_neovius_iwp_lattice() {
	spec := decad.LatticeSpec{
		Min:        r3.Vec{},
		Max:        r3.Vec{X: 20, Y: 20, Z: 20},
		Cell:       units.Millimeters(10),
		Resolution: 16,
	}
	for _, kind := range []struct {
		name  string
		build func(context.Context, decad.LatticeSpec) (*decad.MeshBody, error)
		level float64
	}{
		{"Neovius", decad.NeoviusLattice, 4},
		{"IWP", decad.IWPLattice, 2},
	} {
		spec.Level = units.Scalar(kind.level)
		lattice, err := kind.build(context.Background(), spec)
		if err != nil {
			fmt.Printf("failed to build the %s lattice: %s\n", kind.name, err)
			return
		}
		volume, err := lattice.Volume()
		if err != nil {
			fmt.Printf("failed to measure volume: %s\n", err)
			return
		}
		fmt.Printf("%s: closed: %t, solid fraction: %.2f\n", kind.name, lattice.Report().Closed, volume.Value.Base()/8000)
	}
	// Output:
	// Neovius: closed: true, solid fraction: 0.74
	// IWP: closed: true, solid fraction: 0.55
}

// Infill fills a solid with a lattice and keeps the lattice material inside it, the
// way a slicer fills a print. Here a 10 x 10 x 12 mm block is filled with a gyroid
// repeating every 10 mm, and the infill is lighter than the block.
func Example_infill() {
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to start a sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(5, 5, 15, 15)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(12), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	filled, err := decad.Infill(context.Background(), block, decad.InfillSpec{
		Kind: decad.GyroidInfill, Cell: units.Millimeters(10), Level: units.Scalar(0.8), Resolution: 8})
	if err != nil {
		fmt.Printf("failed to fill: %s\n", err)
		return
	}
	volume, err := filled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("lighter than the block: %t\n", volume.Value.Base() < 1200)
	// Output:
	// lighter than the block: true
}
