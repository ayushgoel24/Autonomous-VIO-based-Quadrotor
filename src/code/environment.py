import yaml

class ApplicationProperties( object ):

    """
    A class to handle application properties stored in a YAML file.

    Attributes:
        _propertiesFile (str): The path to the properties file.
        _applicationProperties (dict): The contents of the properties file.
        propertyCache (dict): A cache to store property values that have already been retrieved.
    """
    
    def __init__( self, propertiesFilePath: str ) -> None:
        """
        Initializes the ApplicationProperties object.

        Args:
            propertiesFilePath (str): The path to the properties file.
        """
        self._propertiesFile = propertiesFilePath
        self._applicationProperties = None
        self.propertyCache = {}

    def initializeProperties( self ) -> dict:
        """
        Initializes the application properties by loading them from the properties file.
        """
        # opens the properties file and load its contents into _applicationProperties
        with open( self._propertiesFile, 'r') as file:
            self._applicationProperties = yaml.safe_load(file)

    def get_property_value( self, propertyName: str ):
        """
        Gets the value of a property with the given name.

        Args:
            propertyName (str): The name of the property to retrieve.

        Returns:
            The value of the property, or None if the property is not found.
        """
        # If the property is already in the cache, return it from the cache
        if propertyName in self.propertyCache:
            return self.propertyCache[ propertyName ]
        
        # If the property is not in the cache, split the property name into a list of properties
        properties = propertyName.split( "." )

        # Traverse the property tree until the desired property is found or an unknown property is encountered
        propertyTree = self._applicationProperties
        for p in properties:
            if p not in propertyTree:
                # If the property is unknown, return None
                return None
            else:
                propertyTree = propertyTree[ p ]

        # If the property is found, return its value
        return propertyTree