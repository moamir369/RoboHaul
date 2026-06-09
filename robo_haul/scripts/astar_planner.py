#!/usr/bin/env python3

import math
import heapq
import yaml
import numpy as np
from PIL import Image

from scipy.ndimage import binary_erosion


class AStarPlanner:
    
    def __init__(self , map_yaml):
        
        with open(map_yaml , 'r') as f:
            cfg = yaml.safe_load(f)

        map_dizini = map_yaml.rsplit('/' , 1)[0]
        image_file = cfg['image']

        if not image_file.startswith('/'):
            image_file =map_dizini + '/' + image_file

        self.resolution = cfg["resolution"]
        ox , oy , gerek_yok  = cfg["origin"]
        self.orjin_x = ox
        self.orjin_y =oy
        self.bos_esik_degeri = cfg.get('free_thresh', 0.196)

        img =Image.open(image_file).convert('L')
        arr = np.array(img, dtype=np.float32) / 255.0
        self.yukseklik, self.genislik = arr.shape
        self.grid = np.flipud(arr)

        self._engel_haritasi_olustur(genisletme_boyutu=8)

    def _engel_haritasi_olustur (self , genisletme_boyutu=8):
            bos_alanlar = self.grid >= self.bos_esik_degeri
            struct =np.ones((2 * genisletme_boyutu + 1, 2 * genisletme_boyutu + 1), dtype=bool)
            self.bos_harita = binary_erosion(bos_alanlar , structure=struct , border_value=0)

    def bos_mu(self ,gx, gy ):

        if gx<0 or gy<0 or gx>=self.genislik or gy>=self.yukseklik :
            return False
        
        return bool(self.bos_harita[gy,gx])
    
    
    def hedef_alani_uygun_mu (self , gx,gy, genisletme_boyutu=4):
        # Sınır dışı kontrolü
        if gx < 0 or gy < 0 or gx >= self.genislik or gy >= self.yukseklik:
            return False
        
        for dx in range(-genisletme_boyutu, genisletme_boyutu + 1):
            for dy in range(-genisletme_boyutu, genisletme_boyutu + 1):
                nx, ny = gx + dx, gy + dy
                
                if nx < 0 or ny < 0 or nx >= self.genislik or ny >= self.yukseklik:
                    return False
                if self.grid[ny, nx] < self.bos_esik_degeri:
                    return False
        return True
    
    def world_to_grid(self, wx, wy):
        gx = int((wx - self.orjin_x) / self.resolution)
        gy = int((wy - self.orjin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.orjin_x + (gx + 0.5) * self.resolution
        wy = self.orjin_y + (gy + 0.5) * self.resolution
        return wx, wy

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    

    def en_yakin_guvenli_hedefi_bul(self, hedef_x, hedef_y, maX_yaricap_m=3.0):
        
        hedef_grid = self.world_to_grid(hedef_x, hedef_y)

        max_yaricap_grid = int(maX_yaricap_m / self.resolution)

        # Hedefin çevresini yarıçapı 1'den başlayarak adım adım tara
        for yaricap in range(1, max_yaricap_grid + 1):
            adaylar = []

            # Daire üzerindeki noktaları kontrol et
            for dx in range(-yaricap, yaricap + 1):
                for dy in range(-yaricap, yaricap + 1):

                    # Sadece dairenin 'çevresindeki' hücreleri tara (içini tarayıp vakit kaybetme)
                    if abs(dx) != yaricap and abs(dy) != yaricap:
                        continue

                    nx = hedef_grid[0] + dx
                    ny = hedef_grid[1] + dy

                    # Hücre güvenli mi?
                    if self.bos_mu(nx, ny):
                        mesafe = math.sqrt(dx**2 + dy**2)
                        adaylar.append((mesafe, nx, ny))

            # Eğer bu yarıçapta uygun hücreler bulduysak, en yakın olanı seç
            if adaylar:
                adaylar.sort() # Mesafeye göre küçükten büyüğe sırala
                _, en_yakin_x, en_yakin_y = adaylar[0]
                
                # Seçilen grid noktasını dünya koordinatına çevir ve döndür
                dunya_x, dunya_y = self.grid_to_world(en_yakin_x, en_yakin_y)
                return dunya_x, dunya_y
        
        # Maksimum arama mesafesi içerisinde hiç güvenli nokta bulunamadıysa
        return None
    

    def plan(self, baslangic_x, baslangic_y, hedef_x, hedef_y, max_geri_donus_m=3.0):
        
        baslangic = self.world_to_grid(baslangic_x, baslangic_y)
        hedef = self.world_to_grid(hedef_x, hedef_y)


        for sisirme_payi in [8, 4, 2, 0]:
            sonuc = self._astar(baslangic, hedef, sisirme_payi)
            if sonuc is not None:
                return sonuc, (hedef_x, hedef_y)
            
        alternatif_hedef = self.en_yakin_guvenli_hedefi_bul(hedef_x, hedef_y, max_geri_donus_m)

        if alternatif_hedef is None:
            return None, None # Hiçbir şekilde gidilecek nokta yok
        
        # 3. Adım: Bulunan alternatif güvenli hedefe rota planla
        alt_hx, alt_hy = alternatif_hedef
        alt_hedef_grid = self.world_to_grid(alt_hx, alt_hy)

        for sisirme_payi in [4, 2, 0]:
            sonuc = self._astar(baslangic, alt_hedef_grid, sisirme_payi)
            if sonuc is not None:
                return sonuc, (alt_hx, alt_hy)
            
        return None, None
    




    def _astar(self, baslangic, hedef, hedef_sisirme):
        

        # 1. Adım: Hedef noktası harita sınırları içinde mi ve güvenli mi?
        if hedef_sisirme == 0:
            if not (0 <= hedef[0] < self.genislik and 0 <= hedef[1] < self.yukseklik):
                return None
            
        else:
            if not self.hedef_alani_uygun_mu(*hedef, genisletme_boyutu=hedef_sisirme):
                return None
            
        # 2. Adım: Arama için hazırlık (Açık liste ve öncelik kuyruğu)
        open_set = []
        heapq.heappush(open_set, (0, baslangic)) # (F-skoru, koordinat)
        gelinen_noktalar = {} # Yolu geri sürmek için (parent map)
        g_skoru = {baslangic: 0} # Başlangıçtan mevcut noktaya olan maliyet

        # 8 yönlü hareket (Sağ, Sol, Yukarı, Aşağı ve çaprazlar)
        komsular = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        # 3. Adım: Ana döngü (Arama)
        while open_set:
            _, suanki = heapq.heappop(open_set)

            # Hedefe yeterince yaklaştıysak (Öklid mesafesi < 2 birim)
            if self.heuristic(suanki, hedef) < 2:
                rota = []
                while suanki in gelinen_noktalar:
                    rota.append(self.grid_to_world(*suanki))
                    suanki = gelinen_noktalar[suanki]
                rota.reverse()
                return rota

            # 4. Adım: Komşu hücreleri değerlendir
            for dx, dy in komsular:
                komsu = (suanki[0] + dx, suanki[1] + dy)

                # Engel kontrolü
                if not self.bos_mu(*komsu):
                    continue

                # Maliyet hesabı: g(n) + adım maliyeti
                hareket_maliyeti = math.sqrt(dx**2 + dy**2)
                yeni_g = g_skoru[suanki] + hareket_maliyeti

                # Daha kısa bir yol bulunduysa güncelle
                if yeni_g < g_skoru.get(komsu, float('inf')):
                    gelinen_noktalar[komsu] = suanki
                    g_skoru[komsu] = yeni_g
                    f = yeni_g + self.heuristic(komsu, hedef) # F = g + h
                    heapq.heappush(open_set, (f, komsu))

        return None
