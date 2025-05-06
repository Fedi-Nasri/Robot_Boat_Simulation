import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import os
from dotenv import load_dotenv

class RTDBStructureChecker:
    """
    A class to connect to Firebase Realtime Database and check for a specific data structure.
    Loads credentials from a .env file.
    """

    def __init__(self, env_path='.env', database_url="https://oceancleaner-741db-default-rtdb.firebaseio.com"):
        """
        Initializes the RTDBStructureChecker.

        Args:
            env_path (str): The path to the .env file. Defaults to '.env'.
                            This file should contain FIREBASE_SERVICE_ACCOUNT_KEY_PATH.
            database_url (str): The URL of your Firebase Realtime Database instance.
                                Defaults to your project's default database URL.
        """
        print(f"--- Initializing RTDB Structure Checker for {database_url} ---")
        # Load environment variables from the specified .env file
        load_dotenv(env_path)

        # Get the path to the service account key file from the environment
        service_account_key_path = os.getenv('FIREBASE_SERVICE_ACCOUNT_KEY_PATH')

        if not service_account_key_path:
            raise EnvironmentError(
                "FIREBASE_SERVICE_ACCOUNT_KEY_PATH not found in your .env file. "
                "Please add the path to your service account key JSON file."
            )
        if not os.path.exists(service_account_key_path):
             raise FileNotFoundError(
                f"Service account key file not found at the path specified in .env: {service_account_key_path}"
            )

        # Store the database URL
        self.database_url = database_url

        try:
            # Initialize the Firebase app using the service account credentials
            # We check if an app is already initialized to avoid errors if this class
            # is instantiated multiple times or in a context where Firebase is already set up.
            # We'll use the default app name if no app exists.
            if not firebase_admin._apps:
                 print("Initializing new Firebase app.")
                 cred = credentials.Certificate(service_account_key_path)
                 firebase_admin.initialize_app(cred, {
                    'databaseURL': self.database_url
                 })
                 print("Firebase app initialized successfully.")
            else:
                 print("Firebase app already initialized.")
                 # If an app exists, we'll just use the existing one.
                 # The db.reference(url=...) method can target the specific URL.

        except Exception as e:
            print(f"Error initializing Firebase app: {e}")
            # Re-raise the exception so the calling code knows initialization failed
            raise

        # Keep a reference to the db module
        self._db = db

    def check_sensors_structure(self):
        """
        Checks if the Firebase Realtime Database at the specified URL
        contains the structure:
        /sensors/gps/time
        /sensors/gps/value
        /sensors/temperature/time
        /sensors/temperature/value

        It checks for the *presence* of these keys, not their data types or values.

        Returns:
            bool: True if the required structure is found under '/sensors', False otherwise.
        """
        # Get a reference to the '/sensors' path in the database
        ref = self._db.reference('sensors', url=self.database_url)
        print(f"Attempting to read data from path '/sensors'...")

        try:
            # Read the data at the /sensors path.
            # shallow=True could be used for a faster check if you only needed
            # to know if the keys (gps, temperature) exist directly under sensors,
            # but to check deeper (time, value), we need the actual data.
            snapshot = ref.get()
            print(f"Read complete. Data found: {snapshot}")

            # Check if the data at /sensors is a dictionary (as expected for nested nodes)
            if not isinstance(snapshot, dict):
                print("Data at '/sensors' is not a dictionary. Structure not found.")
                return False

            sensors_data = snapshot

            # Check if 'gps' and 'temperature' keys exist directly under 'sensors'
            if 'gps' not in sensors_data or 'temperature' not in sensors_data:
                print("Missing 'gps' or 'temperature' key under '/sensors'. Structure not found.")
                return False

            # Get the data for 'gps' and 'temperature'
            gps_data = sensors_data.get('gps')

