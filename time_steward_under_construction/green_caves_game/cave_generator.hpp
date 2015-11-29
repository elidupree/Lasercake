/*

    Copyright Eli Dupree and Isaac Dupree, 2014

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_GREEN_CAVE_GENERATOR_HPP__
#define LASERCAKE_GREEN_CAVE_GENERATOR_HPP__

class cave_generator {
public:

  struct tile_info {
    entity_ID ID;
    bool wall;
  };

private:
  struct tile_info_inner {
    int64_t last_query;
    tile_info info;
    tile_info_inner (): last_query (-1) {}
  };
  struct cave {
  cave(fd_vector center_tile, int64_t radius):center_tile(center_tile),radius(radius){}
  fd_vector center_tile;
int64_t radius;
};
  struct cave_block {
    int64_t last_query;
    std:: vector <cave> caves;
    cave_block (): last_query (-1) {}
  };
public:
  cave_generator (int64_t cache_size =100000, int64_t max_radius_in_tiles = 10, int64_t block_size = 32):
    cache_size (cache_size),
    max_radius_in_tiles (max_radius_in_tiles),
    block_size (block_size), queries (0){}
  
  tile_info const & get (fd_vector tile) const {
    ++ queries;

    tile_info_inner & result = tiles [tile];
    if (result.last_query == -1) {
      result.info.wall = true;
      result.info.ID  = siphash_id::combining('t','i','l','e',tile(0),tile(1));
      auto strategy =rounding_strategy<round_down, negative_continuous_with_positive>(); 
      for (tile_coordinate x = divide(tile(0) - max_radius_in_tiles, block_size, strategy); x*block_size <= tile(0) + max_radius_in_tiles; x++) {
        for (tile_coordinate y = divide (tile(1) - max_radius_in_tiles, block_size, strategy); y*block_size <= tile(1) + max_radius_in_tiles; y++) {
          cave_block & b = caves [fd_vector(x,y)];
          if (b. last_query == -1) {
          
siphash_random_generator rng(siphash_id:: combining (x, y));
    for (tile_coordinate x2 =x* block_size; x2 < (x+1)* block_size; ++x2) {
      for (tile_coordinate y2 = y* block_size; y2 < (y+1)* block_size;++y2) {
        bool any_cave_here = (rng.random_bits(8) == 0) || ((x2 == 0) && (y2 == 0));
        if (any_cave_here) {
int64_t cave_radius = (1ULL <<20)*max_radius_in_tiles /4 + rng.random_bits(20)*3/4;
          assert (cave_radius < max_radius_in_tiles << 20);
          b.caves.emplace_back(fd_vector(x2,y2), cave_radius);
        }
      }
    }

          }
          b. last_query = queries;
          for (cave const& c : b.caves) {
            if ((
                  (c.center_tile(0) - tile(0))*(c.center_tile(0) - tile(0)) +
                  (c.center_tile(1) - tile(1))*(c.center_tile(1) - tile(1))
                ) << 40 <= c.radius*c.radius) {
              result.info.wall = false;
              goto triplebreak;
            }
          }
        }
      }
      triplebreak:;
    }
    result.last_query = queries;    
    if (tiles.size () >cache_size) {
      for (auto iterator = tiles.begin (); iterator != tiles.end ();) {
        if (iterator -> second.last_query <queries - (cache_size/2))
          iterator = tiles.erase (iterator);
        else
          ++iterator;
      }
      
      for (auto iterator = caves.begin (); iterator != caves.end ();) {
        if (iterator -> second.last_query <queries - (cache_size/2))
          iterator = caves.erase (iterator);
        else
          ++iterator;
      }

    }
    return result.info;
  }

private:
  const int64_t cache_size;
  const int64_t max_radius_in_tiles;
  const int64_t block_size;  
  mutable int64_t queries;
  mutable std:: unordered_map <FD_vector, tile_info_inner> tiles;
  mutable std:: unordered_map <FD_vector, cave_block> caves;
};

#endif
