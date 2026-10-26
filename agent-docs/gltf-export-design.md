# glTF export design

`export.GLTF(ctx, w, body, tol, opts...)` writes one body's tessellation as a glTF 2.0 file. It takes the chord
tolerance and `decad.TessellateOption` values of STL, OBJ and 3MF, and defaults to `VerifyNone`. A successful
`Body.Tessellate` result supplies every vertex and triangle. The writer changes no index or winding.

## File

| Decision | Rule |
|---|---|
| F1 | The file is one JSON document ending in a newline. Its fields follow the order of the Go structs in `export/gltf.go`, so equal body, tolerance and options give equal bytes. |
| F2 | `asset` is version `2.0` with generator `decad`. One scene holds one node, which names one mesh with one triangle primitive (`mode` 4). |
| F3 | The primitive has a `POSITION` attribute and 32-bit indices. It has no normals, tangents, colours, materials or textures, so a viewer shades flat from the winding. |
| F4 | One buffer holds the positions, then the indices. It is embedded as a `data:application/octet-stream;base64,` URI, so the file stands alone. |
| F5 | The two buffer views and accessors use the targets and component types glTF names: `ARRAY_BUFFER` and `FLOAT` for `VEC3` positions, `ELEMENT_ARRAY_BUFFER` and `UNSIGNED_INT` for `SCALAR` indices. The position accessor carries `min` and `max`, which glTF requires. |
| F6 | Positions are in metres, as glTF requires. Each millimetre coordinate is multiplied by `0.001` and stored as a 32-bit float, so a stored coordinate differs from the mesh by the float32 rounding of that product. Negative zero is stored as zero. |
| F7 | No axis is exchanged. decad's +Z stays +Z although glTF viewers conventionally draw +Y up. |
| F8 | Triangles keep the mesh's counter-clockwise winding seen from outside, which is glTF's front face. A sheet writes an open mesh. |

## Errors

- A nil context, writer or body is `ErrDegenerate`. A mesh with no vertex or triangle is `ErrDegenerate`.
- Tessellation errors, including a wrong unit kind for `tol` and cancellation, propagate unchanged.
- The document is built before the first write, so a failure in tessellation, encoding or cancellation leaves the
  writer untouched. An I/O error during the single write may leave a partial file.

## Not covered

- Binary `.glb` containers and external `.bin` buffers.
- Normals, materials, colours and per-face groups.
- Scaling to other units or exchanging axes. Transform the body first.
- Reading glTF.

The [glTF 2.0 specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html) defines the JSON schema, the
accessor and buffer view fields, and the metre unit.
