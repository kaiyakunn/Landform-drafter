# grasshopper-landform-tools

A Rhino/Grasshopper Python component that builds a classified landform model from site curves. Give it a site boundary, ridge lines, and pathway and zone outlines at their intended elevations. It returns a terrain mesh, a primary path mesh, and individual patch surfaces for secondary, tertiary, and edge pathways and programmed zones. Everything updates live as the input curves change.


## Why this exists

Testing landform ideas by hand meant modeling terrain, grading paths through it, and re-checking everything after each design change, which took hours per iteration. This tool automates the geometry so designers can move a curve and see the landform update immediately, including on large sites.

## How it works

1. **Sample.** Points are sampled along every input curve. The boundary and ridges use `sample_dist`, and path and zone curves use half that spacing for a tighter fit. Elevations come from the Z values of the input curves.
2. **Triangulate.** All points are tessellated into one master mesh in plan (XY).
3. **Classify.** Each face is assigned by its center point, in priority order: outside boundary (discarded) → programmed area → secondary path → tertiary path → edge path → primary path → terrain.
4. **Rebuild.** Terrain and primary path faces become clean sub-meshes. Each secondary, tertiary, edge, and programmed curve becomes its own Patch Surface Brep, fitted to the mesh vertices inside it.
5. **Smooth.** The terrain and primary path meshes are smoothed and welded.

## Inputs

| Name | Type | Description |
|---|---|---|
| `edges` | List of curves | Site boundary (joined into closed perimeter) |
| `ridges` | List of curves | Internal ridge lines that shape the terrain |
| `paths` | List of closed curves | Primary road/walkway boundaries |
| `second_paths` | List of closed curves | Secondary walkway/trail boundaries |
| `third_paths` | List of closed curves | Tertiary/maintenance pathway boundaries |
| `edge_paths` | List of closed curves | Edge pathway boundaries |
| `programmed_area` | List of closed curves | Programmed zone boundaries |
| `sample_dist` | Float | Point sampling interval along curves (default `1.0`) |
| `smoothness` | Float, 0.0 to 1.0 | Terrain smoothing amount (default `0.0`) |
| `patch_uv_spans` | Integer | U and V span count for patch surfaces (default `10`) |

## Outputs

| Name | Type | Description |
|---|---|---|
| `topo_mesh` | Mesh | Main terrain |
| `path_mesh` | Mesh | Primary pathways |
| `second_path_surf` | List of Breps | One patch surface per secondary pathway |
| `third_path_surf` | List of Breps | One patch surface per tertiary pathway |
| `edge_path_surf` | List of Breps | One patch surface per edge pathway |
| `programmed_surf` | List of Breps | One patch surface per programmed zone |

## Setup

1. Add a **Python Script** component to your Grasshopper canvas.
2. Rename the inputs and outputs to match the tables above.
3. Paste in the contents of `topography_generator.py`.
4. Connect your curves and parameters.

**Requirements:** Rhino with Grasshopper (tested on: *[add Rhino version]*). No external libraries are needed beyond `Rhino.Geometry`.

## Tips and limitations

- Path and zone inputs must be **closed** curves. Open curves are ignored in containment checks.
- Elevation comes entirely from the input curves' Z values, so place curves at their intended heights.
- Faces are classified by their center point, so zone edges follow the mesh resolution. A smaller `sample_dist` gives crisper edges but a slower solve.
- Patch generation checks every face against every zone curve, so runtime grows with site size, mesh density, and zone count.
- Higher `patch_uv_spans` gives tighter patch surfaces but is slower.

## Development notes

The first version of this script was drafted with AI coding assistants and then cross-checked with a second model. I directed the design and checked outputs against real site conditions. Planned improvements include unit tests for the helper functions and sample input files.

## License

MIT. See `LICENSE`.
