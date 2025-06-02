#!/usr/bin/env python3

import rospy
import time
from asv_wave_sim_gazebo.msg import WasteDetection
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest
from std_msgs.msg import String

class ProfessionalWasteCollector:
    def __init__(self):
        rospy.init_node('professional_waste_collector', anonymous=True)

        # Wait for Gazebo services
        try:
            rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
            rospy.wait_for_service('/gazebo/clear_body_wrenches', timeout=10)
        except rospy.ROSException as e:
            rospy.logerr(f"Gazebo services not available: {e}")
            return

        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)

        # PARAMÈTRES PID AVANCÉS POUR RÉGULATION HAUTE PERFORMANCE
        self.kp_angular = 1.2    # Gain proportionnel TRÈS ÉLEVÉ pour réponse rapide
        self.ki_angular = 0.15   # Gain intégral RENFORCÉ pour éliminer l'erreur statique
        self.kd_angular = 0.25   # Gain dérivé ÉLEVÉ pour stabilité maximale

        self.kp_linear = 0.6     # Gain proportionnel RENFORCÉ pour avancement précis
        self.ki_linear = 0.08    # Gain intégral RENFORCÉ pour éliminer l'erreur statique
        self.kd_linear = 0.12    # Gain dérivé RENFORCÉ pour stabilité maximale

        # PARAMÈTRES ADAPTATIFS pour différentes situations
        self.kp_angular_fine = 0.4   # Gains réduits pour ajustements fins
        self.ki_angular_fine = 0.05
        self.kd_angular_fine = 0.08

        self.kp_linear_fine = 0.2    # Gains réduits pour avancement fin
        self.ki_linear_fine = 0.02
        self.kd_linear_fine = 0.04

        # VITESSES POUR ALLER VERS LE DÉCHET
        self.search_speed = 0.4          # Vitesse de recherche (aucun déchet)
        self.approach_speed = 0.15       # Vitesse LENTE pour aller vers le déchet
        self.collection_speed = 0.2      # Vitesse pour collecte finale
        self.regulation_speed = 0.3      # Vitesse de référence pour la régulation

        # SEUILS DE PRÉCISION pour la régulation
        self.centering_threshold = 0.01  # Seuil TRÈS STRICT pour le centrage précis
        self.min_detection_ratio = 0.05  # Ratio minimum pour considérer une détection significative
        self.precision_threshold = 0.02  # Seuil de précision pour les micro-ajustements
        self.fine_tuning_threshold = 0.005  # Seuil ultra-précis pour les ajustements fins
        self.upper_centering_threshold = 0.02  # Seuil très strict pour le centrage dans Upper Middle
        self.upper_centering_time = 0.0  # Temps passé en phase de centrage Upper Middle
        self.min_upper_centering_time = 3.0  # Temps minimum augmenté pour une meilleure stabilisation
        self.min_upper_detection_ratio = 0.8  # Ratio minimum de détection dans Upper Middle (80%)

        # Limites de commande
        self.max_torque = 0.20
        self.min_torque = -0.20
        self.base_speed = -0.30

        # Variables d'état du régulateur AVANCÉ
        self.angular_error_integral = 0.0
        self.linear_error_integral = 0.0
        self.previous_angular_error = 0.0
        self.previous_linear_error = 0.0
        self.previous_time = time.time()

        # FILTRAGE AVANCÉ pour réduire le bruit
        self.angular_error_history = [0.0] * 5  # Historique des 5 dernières erreurs
        self.linear_error_history = [0.0] * 5
        self.angular_derivative_filtered = 0.0
        self.linear_derivative_filtered = 0.0

        # RÉGULATION ADAPTATIVE
        self.regulation_mode = "NORMAL"  # NORMAL, FINE, AGGRESSIVE
        self.error_stability_counter = 0  # Compteur de stabilité
        self.max_error_threshold = 0.1   # Seuil pour mode agressif
        self.fine_error_threshold = 0.02 # Seuil pour mode fin

        # ANTI-WINDUP AVANCÉ
        self.integral_windup_limit = 0.3
        self.derivative_filter_alpha = 0.7  # Facteur de filtrage dérivé (0-1)

        # Identification des ventilateurs
        self.fan_right = "boatcleaningc::fandroit"
        self.fan_left = "boatcleaningc::fangauche"

        # État du système
        self.collection_count = 0
        self.regulation_mode = "SEARCH"  # SEARCH, ORIENT, APPROACH, COLLECT
        self.target_position = {"x": 0, "y": 0}  # Position cible du déchet
        self.regulation_active = False

        # État de navigation : 'search' ou 'collection'
        self.navigation_phase = 'search'  # Phase initiale : recherche

        # Variables manquantes pour compatibilité
        self.strong_turn = -0.15
        self.medium_turn = -0.10
        self.forward_speed = -0.08

        # Seuils de précision pour la détection
        self.precision_threshold = 0.05  # Seuil de précision pour le centrage
        self.approach_threshold = 0.1    # Seuil pour l'approche progressive

        # Taux d'erreur logic
        self.last_detection_time = time.time()
        self.taux_erreur_period = 3.0  # seconds
        self.in_search_state = False
        self.search_start_time = None

        # Subscribe to waste detection
        rospy.Subscriber('/waste_detection', WasteDetection, self.waste_detection_callback)
        # Timer to check detection timeout
        self.taux_erreur_timer = rospy.Timer(rospy.Duration(0.5), self.taux_erreur_check)
        # Subscribe to collection control
        self.collection_control_state = 'start_collection'
        rospy.Subscriber('/collection_control', String, self.collection_control_callback)
        # Publisher for collection status
        self.collection_status_pub = rospy.Publisher('/collection_status', String, queue_size=10)
        rospy.loginfo("Professional Waste Collector with PID Regulation started!")

    def apply_torque(self, link_name, torque):
        try:
            self.clear_wrench(link_name)
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)
                self.apply_wrench(req)
                rospy.loginfo(f"Applied torque {torque} on {link_name}")
            else:
                rospy.loginfo(f"Cleared torque on {link_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to apply torque on {link_name}: {e}")

    def stop_fans(self):
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, 0.0)
        rospy.loginfo("Boat stopped.")

    def move_forward_slowly(self):
        """Move forward slowly for search and approach"""
        self.apply_torque(self.fan_right, self.base_speed)
        self.apply_torque(self.fan_left, self.base_speed)
        rospy.loginfo("Boat moving forward slowly.")

    def calculate_waste_position_error(self, left_count, right_count, middle_count, bottom_middle_count, upper_left_count=0, upper_right_count=0):
        """
        🎯 STRATÉGIE SIMPLE : ALLER VERS LE DÉCHET AVEC RÉGULATION
        Phase 1: Recherche quand aucun déchet
        Phase 2: Aller vers le déchet avec petite vitesse + régulation pour guider vers Middle puis Bottom Middle

        6 ZONES: Left, Right, Upper left, Middle, Upper right, Bottom middle
        """
        # Déterminer si un déchet est détecté
        waste_detected = (left_count > 0 or right_count > 0 or middle_count > 0 or bottom_middle_count > 0 or
                         upper_left_count > 0 or upper_right_count > 0)

        # 🎯 PRIORITÉ 1: DÉCHET EN BOTTOM MIDDLE - COLLECTE FINALE
        if bottom_middle_count > 0:
            angular_error = 0.0
            linear_error = self.collection_speed  # Vitesse pour collecte finale
            rospy.loginfo("🎯 COLLECTE: Déchet en Bottom Middle - COLLECTE FINALE!")
            return angular_error, linear_error

        # 🎯 PRIORITÉ 2: DÉCHET EN MIDDLE - POUSSER VERS BOTTOM MIDDLE
        if middle_count > 0:
            angular_error = 0.0  # Maintenir le cap droit
            linear_error = self.approach_speed  # Vitesse lente pour pousser vers Bottom Middle
            rospy.loginfo("🎯 MIDDLE: Déchet en Middle - poussée vers Bottom Middle")
            return angular_error, linear_error

        # 🎯 DÉCHET DÉTECTÉ - ALLER VERS LUI AVEC RÉGULATION PRÉCISE
        if waste_detected:
            rospy.loginfo("🎯 COLLECTE: Déchet détecté - aller vers lui avec régulation PRÉCISE!")

            # Calculer les ratios pour une régulation précise
            total_detections = left_count + right_count + upper_left_count + upper_right_count + middle_count + bottom_middle_count

            if total_detections > 0:
                left_ratio = (left_count + upper_left_count) / total_detections
                right_ratio = (right_count + upper_right_count) / total_detections

                # Calcul de l'erreur de centrage PRÉCISE
                center_error = left_ratio - right_ratio

                # Guider le déchet vers Middle avec régulation PRÉCISE
                if left_count > 0 or upper_left_count > 0:
                    # Déchet à GAUCHE - Rotation PRÉCISE à gauche pour le guider vers Middle
                    if abs(center_error) > self.precision_threshold:
                        angular_error = 0.15 * center_error  # Correction proportionnelle PRÉCISE
                        rospy.loginfo(f"🔄 GUIDAGE PRÉCIS: Déchet à gauche - erreur={center_error:.3f}, correction={angular_error:.3f}")
                    else:
                        angular_error = 0.05 * center_error  # Micro-correction ultra-précise
                        rospy.loginfo(f"🔄 MICRO-AJUSTEMENT: Déchet à gauche - erreur={center_error:.3f}, micro-correction={angular_error:.3f}")

                    linear_error = self.approach_speed  # Vitesse lente

                elif right_count > 0 or upper_right_count > 0:
                    # Déchet à DROITE - Rotation PRÉCISE à droite pour le guider vers Middle
                    if abs(center_error) > self.precision_threshold:
                        angular_error = 0.15 * center_error  # Correction proportionnelle PRÉCISE
                        rospy.loginfo(f"🔄 GUIDAGE PRÉCIS: Déchet à droite - erreur={center_error:.3f}, correction={angular_error:.3f}")
                    else:
                        angular_error = 0.05 * center_error  # Micro-correction ultra-précise
                        rospy.loginfo(f"🔄 MICRO-AJUSTEMENT: Déchet à droite - erreur={center_error:.3f}, micro-correction={angular_error:.3f}")

                    linear_error = self.approach_speed  # Vitesse lente

                else:
                    # Sécurité - avancement lent avec précision
                    angular_error = 0.0
                    linear_error = self.approach_speed
                    rospy.loginfo("🔄 APPROCHE PRÉCISE: Avancement vers le déchet")
            else:
                # Sécurité - pas de détections valides
                angular_error = 0.0
                linear_error = self.approach_speed
                rospy.loginfo("⚠️ SÉCURITÉ: Aucune détection valide - avancement lent")

            return angular_error, linear_error

        # 🔍 AUCUN DÉCHET - RECHERCHE
        angular_error = 0.0  # Maintenir le cap droit
        linear_error = self.search_speed  # Vitesse de recherche
        rospy.loginfo(f"🔍 RECHERCHE: Avancement à {self.search_speed} - aucun déchet détecté")

        return angular_error, linear_error

    def adaptive_pid_controller(self, angular_error, linear_error):
        """
        RÉGULATEUR PID ADAPTATIF HAUTE PERFORMANCE
        - Régulation adaptative selon l'erreur
        - Filtrage avancé du bruit
        - Anti-windup intelligent
        - Historique des erreurs pour stabilité
        """
        current_time = time.time()
        dt = current_time - self.previous_time

        if dt <= 0:
            dt = 0.01  # Éviter division par zéro

        # === SÉLECTION ADAPTATIVE DES GAINS ===
        angular_error_abs = abs(angular_error)
        linear_error_abs = abs(linear_error)

        # Déterminer le mode de régulation adaptatif
        if angular_error_abs > self.max_error_threshold or linear_error_abs > self.max_error_threshold:
            # ERREUR IMPORTANTE → Mode AGRESSIF
            self.regulation_mode = "AGRESSIVE"
            kp_ang, ki_ang, kd_ang = self.kp_angular, self.ki_angular, self.kd_angular
            kp_lin, ki_lin, kd_lin = self.kp_linear, self.ki_linear, self.kd_linear
            rospy.loginfo("🔥 MODE AGRESSIF: Erreur importante détectée")

        elif angular_error_abs < self.fine_error_threshold and linear_error_abs < self.fine_error_threshold:
            # ERREUR FAIBLE → Mode FIN
            self.regulation_mode = "FINE"
            kp_ang, ki_ang, kd_ang = self.kp_angular_fine, self.ki_angular_fine, self.kd_angular_fine
            kp_lin, ki_lin, kd_lin = self.kp_linear_fine, self.ki_linear_fine, self.kd_linear_fine
            rospy.loginfo("🎯 MODE FIN: Ajustements de précision")

        else:
            # ERREUR NORMALE → Mode NORMAL
            self.regulation_mode = "NORMAL"
            kp_ang = (self.kp_angular + self.kp_angular_fine) / 2
            ki_ang = (self.ki_angular + self.ki_angular_fine) / 2
            kd_ang = (self.kd_angular + self.kd_angular_fine) / 2
            kp_lin = (self.kp_linear + self.kp_linear_fine) / 2
            ki_lin = (self.ki_linear + self.ki_linear_fine) / 2
            kd_lin = (self.kd_linear + self.kd_linear_fine) / 2
            rospy.loginfo("⚖️ MODE NORMAL: Régulation équilibrée")

        # === FILTRAGE AVANCÉ DES ERREURS ===
        # Mise à jour de l'historique des erreurs
        self.angular_error_history.pop(0)
        self.angular_error_history.append(angular_error)
        self.linear_error_history.pop(0)
        self.linear_error_history.append(linear_error)

        # Filtrage par moyenne mobile pour réduire le bruit
        angular_error_filtered = sum(self.angular_error_history) / len(self.angular_error_history)
        linear_error_filtered = sum(self.linear_error_history) / len(self.linear_error_history)

        # === RÉGULATION ANGULAIRE AVANCÉE ===
        # Terme proportionnel avec erreur filtrée
        angular_p = kp_ang * angular_error_filtered

        # Terme intégral avec anti-windup intelligent
        self.angular_error_integral += angular_error_filtered * dt
        # Anti-windup conditionnel
        if abs(self.angular_error_integral) > self.integral_windup_limit:
            self.angular_error_integral = self.integral_windup_limit * (1 if self.angular_error_integral > 0 else -1)
            rospy.loginfo("⚠️ ANTI-WINDUP: Limitation intégrale angulaire")
        angular_i = ki_ang * self.angular_error_integral

        # Terme dérivé avec filtrage avancé
        angular_derivative_raw = (angular_error_filtered - self.previous_angular_error) / dt
        # Filtrage passe-bas du terme dérivé
        self.angular_derivative_filtered = (self.derivative_filter_alpha * self.angular_derivative_filtered +
                                          (1 - self.derivative_filter_alpha) * angular_derivative_raw)
        angular_d = kd_ang * self.angular_derivative_filtered

        # Commande angulaire totale
        angular_command = angular_p + angular_i + angular_d

        # === RÉGULATION LINÉAIRE AVANCÉE ===
        # Terme proportionnel avec erreur filtrée
        linear_p = kp_lin * linear_error_filtered

        # Terme intégral avec anti-windup intelligent
        self.linear_error_integral += linear_error_filtered * dt
        # Anti-windup conditionnel
        if abs(self.linear_error_integral) > self.integral_windup_limit:
            self.linear_error_integral = self.integral_windup_limit * (1 if self.linear_error_integral > 0 else -1)
            rospy.loginfo("⚠️ ANTI-WINDUP: Limitation intégrale linéaire")
        linear_i = ki_lin * self.linear_error_integral

        # Terme dérivé avec filtrage avancé
        linear_derivative_raw = (linear_error_filtered - self.previous_linear_error) / dt
        # Filtrage passe-bas du terme dérivé
        self.linear_derivative_filtered = (self.derivative_filter_alpha * self.linear_derivative_filtered +
                                         (1 - self.derivative_filter_alpha) * linear_derivative_raw)
        linear_d = kd_lin * self.linear_derivative_filtered

        # Commande linéaire totale
        linear_command = linear_p + linear_i + linear_d

        # === SATURATION INTELLIGENTE ===
        angular_command = max(min(angular_command, self.max_torque), self.min_torque)
        linear_command = max(min(linear_command, self.max_torque), self.min_torque)

        # Mise à jour des variables d'état
        self.previous_angular_error = angular_error_filtered
        self.previous_linear_error = linear_error_filtered
        self.previous_time = current_time

        # === LOGS AVANCÉS ===
        rospy.loginfo(f"🚀 PID ADAPTATIF [{self.regulation_mode}]: Erreur Ang={angular_error:.4f}→{angular_error_filtered:.4f}")
        rospy.loginfo(f"🚀 PID ADAPTATIF [{self.regulation_mode}]: Erreur Lin={linear_error:.4f}→{linear_error_filtered:.4f}")
        rospy.loginfo(f"🎯 COMMANDES: Angular P={angular_p:.4f} I={angular_i:.4f} D={angular_d:.4f} = {angular_command:.4f}")
        rospy.loginfo(f"🎯 COMMANDES: Linear P={linear_p:.4f} I={linear_i:.4f} D={linear_d:.4f} = {linear_command:.4f}")

        return angular_command, linear_command

    def apply_differential_control(self, angular_command, linear_command):
        """
        COMMANDE DIFFÉRENTIELLE PROFESSIONNELLE
        Convertit les commandes angulaire/linéaire en commandes moteur
        """
        # Calcul des vitesses moteur différentielles
        right_motor = self.base_speed + linear_command - angular_command
        left_motor = self.base_speed + linear_command + angular_command

        # Saturation des moteurs
        right_motor = max(min(right_motor, self.max_torque), self.min_torque)
        left_motor = max(min(left_motor, self.max_torque), self.min_torque)

        rospy.loginfo(f"PID Control: Angular={angular_command:.3f}, Linear={linear_command:.3f}")
        rospy.loginfo(f"Motors: Right={right_motor:.3f}, Left={left_motor:.3f}")

        return right_motor, left_motor

    def regulate_towards_waste(self, left_count, right_count, middle_count, bottom_middle_count, upper_left_count=0, upper_right_count=0):
        """
        RÉGULATION PROFESSIONNELLE POUR MAINTENIR LE DÉCHET EN MIDDLE
        Objectif: Orienter le bateau pour que le déchet soit dans la zone MIDDLE
        6 ZONES: Left, Right, Upper left, Middle, Upper right, Bottom middle
        """

        # OBJECTIF PRINCIPAL: Déchet en Bottom Middle = CONTINUER L'AVANCEMENT POUR RAMASSER
        if bottom_middle_count >= 1:
            rospy.loginfo("🎯 RAMASSAGE: Déchet en Bottom Middle - AVANCEMENT POUR COLLECTE!")
            self.move_forward_slowly()
            rospy.sleep(5)
            self.stop_fans()
            rospy.loginfo("Bateau arrêté définitivement après collecte en Bottom Middle.")
            rospy.signal_shutdown("Collecte terminée.")
            return True

        # STRATÉGIE: Calculer l'erreur pour amener le déchet vers MIDDLE
        angular_error, linear_error = self.calculate_waste_position_error(
            left_count, right_count, middle_count, bottom_middle_count, upper_left_count, upper_right_count
        )

        # RÉGULATION PID PROFESSIONNELLE
        angular_command, linear_command = self.adaptive_pid_controller(angular_error, linear_error)

        # APPLICATION DE LA COMMANDE DIFFÉRENTIELLE
        right_motor, left_motor = self.apply_differential_control(angular_command, linear_command)

        # APPLIQUER LES COMMANDES AUX MOTEURS
        self.apply_torque(self.fan_right, right_motor)
        self.apply_torque(self.fan_left, left_motor)

        # AFFICHAGE DU STATUT D'ORIENTATION VERS LE DÉCHET
        rospy.loginfo(f" ORIENTATION VERS DÉCHET: Angular={angular_error:.2f}, Linear={linear_error:.2f}")
        rospy.loginfo(f" COMMANDES MOTEUR: Droite={right_motor:.3f}, Gauche={left_motor:.3f}")

        # DÉTERMINER LE TYPE D'ORIENTATION EN COURS
        waste_detected = (left_count > 0 or right_count > 0 or middle_count > 0)

        if waste_detected:
            if left_count >= 1 and right_count == 0:
                if middle_count >= 1:
                    rospy.loginfo("ORIENTATION: Déchet à gauche + centre - rotation modérée VERS LA GAUCHE")
                else:
                    rospy.loginfo(" ORIENTATION: Déchet à gauche - rotation forte VERS LA GAUCHE")
            elif right_count >= 1 and left_count == 0:
                if middle_count >= 1:
                    rospy.loginfo("ORIENTATION: Déchet à droite + centre - rotation modérée VERS LA DROITE")
                else:
                    rospy.loginfo("ORIENTATION: Déchet à droite - rotation forte VERS LA DROITE")
            elif middle_count >= 1:
                rospy.loginfo("ORIENTATION: Déchet centré - guidage direct vers Bottom Middle")
            elif left_count >= 1 and right_count >= 1:
                rospy.loginfo(" ORIENTATION: Déchet des deux côtés - maintien du cap central")
        else:
            rospy.loginfo(" RECHERCHE: Aucun déchet détecté - avancement de recherche")

        self.regulation_active = True
        return False

    def waste_detection_callback(self, msg):
        # Initialize counts for the 6 zones exactly
        middle_count = 0
        bottom_middle_count = 0
        left_count = 0
        right_count = 0
        upper_left_count = 0
        upper_right_count = 0

        for section in msg.sections:
            if section.section == "Middle":
                middle_count = section.count
            elif section.section == "Bottom middle":
                bottom_middle_count = section.count
            elif section.section == "Left":
                left_count = section.count
            elif section.section == "Right":
                right_count = section.count
            elif section.section == "Upper left":
                upper_left_count = section.count
            elif section.section == "Upper right":
                upper_right_count = section.count

        rospy.loginfo(f"🎯 DÉTECTION 6 ZONES: Left={left_count}, Right={right_count}, Middle={middle_count}, Bottom Middle={bottom_middle_count}")
        rospy.loginfo(f"🎯 ZONES UPPER: Upper Left={upper_left_count}, Upper Right={upper_right_count}")

        # Déterminer si un déchet est détecté
        waste_detected = (left_count > 0 or right_count > 0 or middle_count > 0 or bottom_middle_count > 0 or upper_left_count > 0 or upper_right_count > 0)
        if waste_detected:
            self.last_detection_time = time.time()
            if self.in_search_state:
                rospy.loginfo("Déchet détecté, sortie du mode recherche.")
                self.in_search_state = False
                self.search_start_time = None

        # SYSTÈME DE RÉGULATION INTELLIGENT - Focus sur maintenir en MIDDLE
        collected = self.regulate_towards_waste(left_count, right_count, middle_count, bottom_middle_count, upper_left_count, upper_right_count)

        if collected:
            rospy.loginfo(f"Total waste collected: {self.collection_count}")
            # Continue moving forward for 5 seconds after collection
            rospy.loginfo("Continuing in the same direction for 5 seconds...")
            self.move_forward_slowly()
            rospy.sleep(5)  # Attendre 5 secondes
            self.stop_fans()
            rospy.loginfo("Boat stopped definitively after collection.")
            rospy.signal_shutdown("Waste collection completed.")

    def taux_erreur_check(self, event):
        now = time.time()
        if not self.in_search_state:
            if now - self.last_detection_time > self.taux_erreur_period:
                rospy.logwarn("Aucun déchet détecté depuis 3s, passage en mode recherche lente.")
                self.in_search_state = True
                self.search_start_time = now
                self.move_forward_slowly()
        else:
            # Already in search state
            if self.search_start_time and (now - self.search_start_time > self.taux_erreur_period):
                rospy.logwarn("Toujours aucun déchet après recherche lente, arrêt des ventilateurs.")
                self.stop_fans()
                self.in_search_state = False
                self.search_start_time = None

    def collection_control_callback(self, msg):
        self.collection_control_state = msg.data
        rospy.loginfo(f"[ProfessionalWasteCollector] Collection control state: {self.collection_control_state}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.collection_control_state == 'end_collection':
                self.collection_status_pub.publish('pause')
                rospy.loginfo("[ProfessionalWasteCollector] Paused by /collection_control. Waiting for 'start_collection'...")
                while self.collection_control_state == 'end_collection' and not rospy.is_shutdown():
                    rate.sleep()
                rospy.loginfo("[ProfessionalWasteCollector] Resumed by /collection_control.")
            self.collection_status_pub.publish('collecting')
            rospy.spin()

if __name__ == '__main__':
    try:
        node = ProfessionalWasteCollector()
        node.run()
    except rospy.ROSInterruptException:
        pass

