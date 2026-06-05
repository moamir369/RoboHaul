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

        if gx<0 or gy<0 or gx>=self.yukseklik or gy>=self.genislik :
            return False
        
        return bool(self.bos_harita[gy,gx])

    def esnetilmis_bos_mu(self, hücre_x, hücre_y, genisletme_boyutu=4):
        """Sadece hedef için — Eğer hedef duvara yakınsa daha küçük bir genisletme_boyutu kullanılır"""
        if hücre_x < 0 or hücre_y < 0 or hücre_x >= self.genislik or hücre_y >= self.yukseklik:
            return False
        for degisim_x in range(-genisletme_boyutu, genisletme_boyutu + 1):
            for degisim_y in range(-genisletme_boyutu, genisletme_boyutu + 1):
                komsu_x, komsu_y = hücre_x + degisim_x, hücre_y + degisim_y
                if komsu_x < 0 or komsu_y < 0 or komsu_x >= self.genislik or komsu_y >= self.yukseklik:
                    return False
                if self.grid[komsu_y, komsu_x] < self.bos_esik_degeri:
                    return False
        return True
    
    def dunya_to_harita(self, dunya_x, dunya_y):
        hucre_x = int((dunya_x - self.orjin_x) / self.resolution)
        hucre_y = int((dunya_y - self.orjin_y) / self.resolution)
        return hucre_x, hucre_y

    def harita_to_dunya(self, hucre_x, hucre_y):
        dunya_x = self.orjin_x + (hucre_x + 0.5) * self.resolution
        dunya_y = self.orjin_y + (hucre_y + 0.5) * self.resolution
        return dunya_x, dunya_y

    def sezgisel_maliyet(self, nokta_a, nokta_b):
        return math.sqrt((nokta_a[0] - nokta_b[0]) ** 2 + (nokta_a[1] - nokta_b[1]) ** 2)
    
    def en_yakin_bos_hedefi_bul(self, dunya_x, dunya_y, maks_yari_cap_m=3.0):

        hedef_hucre = self.dunya_to_harita(dunya_x, dunya_y)

        # Maksimum mesafeyi hücre sayısına dönüştür
        maks_yari_cap_hucre = int(maks_yari_cap_m / self.resolution)

        # 1 hücre yarıçaptan başlayarak dışarıya doğru genişle
        for yari_cap in range(1, maks_yari_cap_hucre + 1):

            adaylar = []

            # Mevcut çemberin çevresindeki tüm hücreleri tara
            for degisim_x in range(-yari_cap, yari_cap + 1):
                for degisim_y in range(-yari_cap, yari_cap + 1):

                    # Sadece çemberin en dış sınırındaki hücreleri al (iç kısmı atla)
                    if abs(degisim_x) != yari_cap and abs(degisim_y) != yari_cap:
                        continue

                    komsu_x = hedef_hucre[0] + degisim_x
                    komsu_y = hedef_hucre[1] + degisim_y

                    if self.bos_mu(komsu_x, komsu_y):
                        mesafe = math.sqrt(degisim_x * degisim_x + degisim_y * degisim_y)
                        adaylar.append((mesafe, komsu_x, komsu_y))

            if adaylar:
                # Bu çember üzerindeki en yakın boş noktayı seç
                adaylar.sort()
                _, en_iyi_x, en_iyi_y = adaylar[0]
                dunya_sonuc_x, dunya_sonuc_y = self.harita_to_dunya(en_iyi_x, en_iyi_y)
                return dunya_sonuc_x, dunya_sonuc_y

        # Belirtilen maks_yari_cap_m içinde hiçbir boş nokta bulunamadı
        return None
    
    def planla(self, baslangic_x, baslangic_y, hedef_x, hedef_y, maks_yedek_mesafe_m=3.0):

        baslangic_hucre = self.dunya_to_harita(baslangic_x, baslangic_y)
        hedef_hucre     = self.dunya_to_harita(hedef_x, hedef_y)

        #  1 Kademeli Genişletme (Inflate) Değerleri ile Orijinal Hedef 
        for hedef_genisletme in [8, 4, 2, 0]:
            sonuc = self._astar(baslangic_hucre, hedef_hucre, hedef_genisletme)
            if sonuc is not None:
                return sonuc, (hedef_x, hedef_y)

        #  2 Genişleyen Çemberlerle Alternatif Nokta Arama 
        yedek_hedef = self.en_yakin_bos_hedefi_bul(hedef_x, hedef_y, maks_yari_cap_m=maks_yedek_mesafe_m)

        if yedek_hedef is None:
            return None, None

        yedek_x, yedek_y    = yedek_hedef
        yedek_hedef_hucre   = self.dunya_to_harita(yedek_x, yedek_y)

        for hedef_genisletme in [4, 2, 0]:
            sonuc = self._astar(baslangic_hucre, yedek_hedef_hucre, hedef_genisletme)
            if sonuc is not None:
                return sonuc, (yedek_x, yedek_y)

        return None, None
    

    def _astar(self, baslangic, hedef, hedef_genisletme):

        if hedef_genisletme == 0:
            if not (0 <= hedef[0] < self.genislik and 0 <= hedef[1] < self.yukseklik):
                return None
        else:
            if not self.esnetilmis_bos_mu(*hedef, genisletme_boyutu=hedef_genisletme):
                return None

        acik_liste = []
        heapq.heappush(acik_liste, (0, baslangic))
        gelinen_yer = {}
        g_skoru     = {baslangic: 0}

        # Robotun hareket edebileceği 8 yön (Yatay, Dikey ve Çaprazlar)
        komsular = [
            (1,  0), (-1,  0), (0,  1), (0, -1),
            (1,  1), (1,  -1), (-1, 1), (-1, -1)
        ]

        while acik_liste:

            _, mevcut_hucre = heapq.heappop(acik_liste)

            # Hedefe yeterince yaklaşıldıysa (Mesafe 2 hücreden azsa) yolu geri sararak oluştur
            if self.sezgisel_maliyet(mevcut_hucre, hedef) < 2:
                yol = []
                while mevcut_hucre in gelinen_yer:
                    yol.append(self.harita_to_dunya(*mevcut_hucre))
                    mevcut_hucre = gelinen_yer[mevcut_hucre]
                yol.reverse()
                return yol

            for degisim_x, degisim_y in komsular:
                komsu_hucre = (mevcut_hucre[0] + degisim_x, mevcut_hucre[1] + degisim_y)

                if not self.bos_mu(*komsu_hucre):
                    continue

                hareket_maliyeti = math.sqrt(degisim_x * degisim_x + degisim_y * degisim_y)
                yeni_g_skoru = g_skoru[mevcut_hucre] + hareket_maliyeti

                if yeni_g_skoru < g_skoru.get(komsu_hucre, 999999):
                    g_skoru[komsu_hucre]  = yeni_g_skoru
                    gelinen_yer[komsu_hucre] = mevcut_hucre
                    f_skoru = yeni_g_skoru + self.sezgisel_maliyet(komsu_hucre, hedef)
                    heapq.heappush(acik_liste, (f_skoru, komsu_hucre))

        return None
    
