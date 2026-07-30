# Autonomous Logistics for Nebraska Pharmaceutical Deserts
 
Code and data for the paper *Where Pharmaceutical Deserts Are and How to Serve Them: A Geospatial Index and Autonomous Routing Study in Rural Nebraska*.
 
The study identifies **pharmaceutical deserts** in Nebraska using a
Pharmaceutical Desert Index (distance to the nearest pharmacy, distance to the
nearest hospital, and social vulnerability), then evaluates delivery options including conventional **truck-only** and
**truck-and-drone** system and quantifies energy, cost and time impacts on the desert and household levels.
 
## Repository structure
 
- **`GIS_Work/`** — where the GIS and map work in the paper can be found:
  identifying the pharmaceutical deserts and producing the maps.
- **`Delivery_Optimization/`** — the delivery optimization: siting drone-launch
  stops, routing the truck-only and truck-and-drone systems, generating the
  routes, and computing per-household energy and cost. See the README inside that
  folder for how to run it.
