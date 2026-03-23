import os
import googlemaps
from googlemaps.exceptions import ApiError
from datetime import datetime

# ✅ Put your key in Windows env first (recommended):
# setx GMAPS_API_KEY "YOUR_KEY"
API_KEY = os.getenv("GMAPS_API_KEY")

if not API_KEY:
    raise RuntimeError("GMAPS_API_KEY not found in environment. Set it first.")

gmaps = googlemaps.Client(key=API_KEY)

locations = [
    "Shahbagh Circle, Dhaka",
    "Bangla Academy, Dhaka",
    "Dhaka University TSC, Dhaka",
    "National Museum, Shahbagh, Dhaka",
    "BSMMU PG Hospital, Dhaka",
    "BIRDEM Hospital, Dhaka",
    "Katabon, Dhaka"
]

try:
    matrix = gmaps.distance_matrix(
        origins=locations,
        destinations=locations,
        mode="driving",
        departure_time="now"   # traffic-aware
    )

except ApiError as e:
    # ✅ This is where Billing / API enable issues show up
    msg = str(e)
    if "You must enable Billing" in msg or "REQUEST_DENIED" in msg:
        raise RuntimeError(
            "Google returned REQUEST_DENIED.\n"
            "Fix:\n"
            "1) Enable Billing on your Google Cloud project\n"
            "2) Enable Distance Matrix API\n"
            "3) Ensure API key restrictions allow Distance Matrix API\n"
            f"\nOriginal error: {e}"
        )
    raise

except Exception as e:
    raise RuntimeError(f"Google Maps request failed: {e}")

# top-level status
if matrix.get("status") != "OK":
    raise RuntimeError(f"Distance Matrix failed. status={matrix.get('status')} full={matrix}")

# Build directed graph
graph = {i: {} for i in range(len(locations))}

for i in range(len(locations)):
    elements = matrix["rows"][i]["elements"]
    for j in range(len(locations)):
        if i == j:
            continue

        elem = elements[j]
        if elem.get("status") != "OK":
            print(f"Skipping {i}->{j}, element status={elem.get('status')}")
            continue

        distance_m = elem["distance"]["value"]
        travel_time_s = elem.get("duration_in_traffic", elem["duration"])["value"]

        graph[i][j] = {"distance": distance_m, "time": travel_time_s}

print("Graph built successfully.")
print("Example edge 0->1:", graph.get(0, {}).get(1))
