package export

import (
	"archive/zip"
	"bytes"
	"context"
	"encoding/xml"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const threeMFContentTypes = `<?xml version="1.0" encoding="UTF-8"?>` + "\n" +
	`<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">` +
	`<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>` +
	`<Override PartName="/3D/3dmodel.model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>` +
	`</Types>`

const threeMFRelationships = `<?xml version="1.0" encoding="UTF-8"?>` + "\n" +
	`<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">` +
	`<Relationship Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" ` +
	`Target="/3D/3dmodel.model"/></Relationships>`

// ThreeMF writes one body as a deterministic 3MF Core package at the requested
// chord tolerance. Solids become model objects; sheets become surface objects.
// The mesh defaults to decad.VerifyNone. Pass decad.WithVerification to request
// stronger proofs. ctx, w, and body must not be nil.
func ThreeMF(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: 3MF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: 3MF: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}

	return writeThreeMFPackage(ctx, w, []threeMFObject{{kind: body.Kind(), vertices: mesh.Vertices(),
		triangles: mesh.Triangles()}})
}

// threeMFObject is one mesh object of a package. A name is written when set.
type threeMFObject struct {
	name      string
	kind      decad.BodyKind
	vertices  []r3.Vec
	triangles [][3]int
}

// writeThreeMFPackage builds the whole archive before it writes to w, so a
// failure before the copy leaves w untouched.
func writeThreeMFPackage(ctx context.Context, w io.Writer, objects []threeMFObject) error {
	var archive bytes.Buffer
	zw := zip.NewWriter(&archive)
	if err := writeThreeMFPart(zw, "[Content_Types].xml", threeMFContentTypes); err != nil {
		return err
	}
	if err := writeThreeMFPart(zw, "_rels/.rels", threeMFRelationships); err != nil {
		return err
	}
	model, err := zw.Create("3D/3dmodel.model")
	if err != nil {
		return fmt.Errorf("export: 3MF model part: %w", err)
	}
	if err := writeThreeMFModel(ctx, model, objects); err != nil {
		return err
	}
	if err := zw.Close(); err != nil {
		return fmt.Errorf("export: 3MF archive: %w", err)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	data := archive.Bytes()
	for len(data) > 0 {
		if err := ctx.Err(); err != nil {
			return err
		}
		chunk := data
		if len(chunk) > 32*1024 {
			chunk = chunk[:32*1024]
		}
		n, err := w.Write(chunk)
		if ctxErr := ctx.Err(); ctxErr != nil {
			return ctxErr
		}
		if err != nil {
			return fmt.Errorf("export: 3MF write failed: %w", err)
		}
		if n != len(chunk) {
			return fmt.Errorf("export: 3MF write failed: %w", io.ErrShortWrite)
		}
		data = data[n:]
	}
	return ctx.Err()
}

func writeThreeMFPart(zw *zip.Writer, name, content string) error {
	w, err := zw.Create(name)
	if err != nil {
		return fmt.Errorf("export: 3MF part %s: %w", name, err)
	}
	if _, err := io.WriteString(w, content); err != nil {
		return fmt.Errorf("export: 3MF part %s: %w", name, err)
	}
	return nil
}

func writeThreeMFModel(ctx context.Context, w io.Writer, objects []threeMFObject) error {
	sw := stickyWriter{w: w}
	sw.printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
	sw.printf("<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n")
	sw.printf("<resources>")
	for i, obj := range objects {
		objectType := "model"
		if obj.kind == decad.BodySheet {
			objectType = "surface"
		}
		name := ""
		if obj.name != "" {
			var esc bytes.Buffer
			if err := xml.EscapeText(&esc, []byte(obj.name)); err != nil {
				return fmt.Errorf("export: 3MF object name: %w", err)
			}
			name = fmt.Sprintf(" name=\"%s\"", esc.String())
		}
		sw.printf("<object id=\"%d\" type=\"%s\"%s><mesh><vertices>\n", i+1, objectType, name)
		for _, v := range obj.vertices {
			if err := ctx.Err(); err != nil {
				return err
			}
			sw.printf("<vertex x=\"%s\" y=\"%s\" z=\"%s\"/>\n", fmtCoord(v.X), fmtCoord(v.Y), fmtCoord(v.Z))
			if sw.err != nil {
				return fmt.Errorf("export: 3MF model part: %w", sw.err)
			}
		}
		sw.printf("</vertices><triangles>\n")
		for _, tri := range obj.triangles {
			if err := ctx.Err(); err != nil {
				return err
			}
			sw.printf("<triangle v1=\"%d\" v2=\"%d\" v3=\"%d\"/>\n", tri[0], tri[1], tri[2])
			if sw.err != nil {
				return fmt.Errorf("export: 3MF model part: %w", sw.err)
			}
		}
		sw.printf("</triangles></mesh></object>")
	}
	sw.printf("</resources><build>")
	for i := range objects {
		sw.printf("<item objectid=\"%d\"/>", i+1)
	}
	sw.printf("</build></model>\n")
	if sw.err != nil {
		return fmt.Errorf("export: 3MF model part: %w", sw.err)
	}
	return nil
}
