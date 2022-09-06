import geopandas

path_to_data = geopandas.datasets.get_path("nybb")
gdf = geopandas.read_file(path_to_data)


gdf.to_file("my_file.geojson", driver="GeoJSON")
gdf = gdf.set_index("BoroName")
gdf["area"] = gdf.area
gdf['boundary'] = gdf.boundary
gdf['centroid'] = gdf.centroid
first_point = gdf['centroid'].iloc[0]
gdf['distance'] = gdf['centroid'].distance(first_point)
gdf['distance'].mean()
gdf.plot("area", legend=True)

